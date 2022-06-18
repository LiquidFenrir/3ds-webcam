#include <3ds.h>
#include <citro3d.h>
#include <citro2d.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <span>
#include <array>
#include <algorithm>
#include "ctr_thread.h"
#include "sprites.h"

#define QOI_IMPLEMENTATION
#include "qoi_mod.h"

#define SOC_ALIGN       0x1000
#define SOC_BUFFERSIZE  0x100000
#define MIC_ALIGN       0x1000
#define MIC_BUFFERSIZE  0x30000

extern "C" {

#include <unistd.h>
#include <malloc.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

}

// #define fprintf(a, b, ...) __VA_ARGS__

char errbuf[1024] = {0};
static void show_error()
{
	fputs(errbuf, stderr);

	errorConf err;
	errorInit(&err, errorType::ERROR_TEXT_WORD_WRAP, CFG_Language::CFG_LANGUAGE_EN);
	errorText(&err, errbuf);
	errorDisp(&err);
}

static u32 *SOC_buffer = nullptr;
static u8 *MIC_buffer = nullptr;
static u32 micbuf_datasize = 0;
static bool cam_on = true;
static bool mic_on = true;

s32 sock = -1, csock = -1;

template<class T, std::size_t N = 4>
struct DoubleBuffer {
	static constexpr inline auto NUM_BUFS = N;
	std::array<T, NUM_BUFS> arr;
	std::array<u32, NUM_BUFS> sizes;
	std::array<bool, NUM_BUFS> written;
	unsigned char current_writing{0};
	unsigned char current_reading{0};

	DoubleBuffer()
	{
		written.fill(false);
	}

	bool can_read() const
	{
		return written[current_writing];
	}
	bool can_write() const
	{
		return !written[current_writing];
	}

	void next_write()
	{
		written[current_writing] = true;
		current_writing = (current_writing + 1) % NUM_BUFS;
	}
	void next_write_sz(const u32 sz)
	{
		sizes[current_writing] = sz;
		next_write();
	}

	auto get_reading()
	{
		return std::span(arr[current_reading], sizes[current_reading]);
	}

	void next_read()
	{
		written[current_reading] = false;
		current_reading = (current_reading + 1) % NUM_BUFS;
	}
};

using VideoBuffer_t = DoubleBuffer<unsigned char*, 2>;
using AudioBuffer_t = DoubleBuffer<u8*, 4>;

static void send_data(std::span<const std::byte> data)
{
	const auto pack_size = 2048;
	const auto d = std::ldiv(data.size_bytes(), pack_size);
	const auto out = htonl(data.size_bytes());
	send(csock, &out, sizeof(out), 0);
	// svcSleepThread(1);
	const auto* ptr = data.data();
	for(long i = 0; i < d.quot; ++i)
	{
		send(csock, ptr, pack_size, 0);
		ptr += pack_size;
		// svcSleepThread(1);
	}
	if(d.rem)
	{
		send(csock, ptr, d.rem, 0);
	}
}

static void send_thread_func(Handle event_stop, ctr::mutex& sockets_mutex, VideoBuffer_t& videobuffer, AudioBuffer_t& audiobuffer)
{
	bool cancel = false;
	int cnt = 0;
	while(!cancel)
	{
		Result res = 0;
		res = svcWaitSynchronization(event_stop, 10 * 1000ll * 1000);
		cancel = (res == 0);
		if(cancel)
		{
			break;
		}

		std::unique_lock sock_lock(sockets_mutex);
		if(sock > 0 && csock > 0)
		{
			cnt++;
			if(cnt == 100)
			{
				fprintf(stderr, "check send 100th\n");
				cnt = 0;
			}
			{
				if(cam_on && videobuffer.can_read())
				{
					const auto data = videobuffer.get_reading();
					fprintf(stderr, "send video: %zd bytes\n", data.size_bytes());
					send_data(std::as_bytes(std::span(data)));
					videobuffer.next_read();
				}
			}
			{
				if(mic_on && audiobuffer.can_read())
				{
					const auto data = audiobuffer.get_reading();
					fprintf(stderr, "send audio: %zd bytes\n", data.size_bytes());
					send_data(std::as_bytes(std::span(data)));
					audiobuffer.next_read();
				}
			}
		}
	}
}
static void mic_thread_func(Handle event_stop, AudioBuffer_t& audiobuffer)
{
	Handle handles[2];
	handles[0] = event_stop;

	bool cancel = false;
	bool have_data = false;
	MICU_GetEventHandle(&handles[1]);
	const auto dsize = std::min<u32>(micbuf_datasize / 2, audiobuffer.sizes[0] - 4);
	const u32 dataoffs[2] = {
		0,
		dsize
	};
	int off = 0;
	fprintf(stderr, "micbuf_datasize: %ld : %ld\n", micbuf_datasize, dsize);
	while(!cancel)
	{
		// fprintf(stderr, "MICU_StartSampling\n");
		MICU_StartSampling(MICU_ENCODING_PCM16_SIGNED, MICU_SAMPLE_RATE_32730, dataoffs[off], dsize, false);
		off ^= 1;

		if(have_data && csock >= 0 && mic_on && audiobuffer.can_write())
		{
			memcpy(audiobuffer.arr[audiobuffer.current_writing] + 4, MIC_buffer + dataoffs[off], dsize);
			audiobuffer.next_write();
			have_data = false;
		}

		s32 index = -1;
		Result res = svcWaitSynchronizationN(&index, handles, 2, false, U64_MAX);

		// fprintf(stderr, "svcWaitSynchronizationN %08lx, idx %ld\n", res, index);
		if(R_SUCCEEDED(res))
		{
			if(index == 0)
			{
				fprintf(stderr, "get cancelled MIC!\n");
				MICU_StopSampling();
				cancel = true;
				break;
			}
			else
			{
				have_data = true;
			}
		}
		else
		{
			MICU_StopSampling();
		}
	}

	// fprintf(stderr, "micu stoppu\n");
	// // fprintf(stderr, "closing mic handle: %08lx\n", svcCloseHandle(handles[1]));
}
static void capture_cam_thread_func(Handle event_stop, bool& using_front, const qoi_desc& desc, VideoBuffer_t& videobuffer)
{
	Handle camReceiveEvent[4] = {0};
	camReceiveEvent[0] = event_stop;

	u32 transferUnit = 0;
	const u32 bufsz = desc.width * desc.height;
	auto screensz = SIZE_DS_LCD;
	if(desc.width == 640 && desc.height == 480)
	{
		screensz = SIZE_VGA;
	}
	else if(desc.width == 400 && desc.height == 240)
	{
		screensz = SIZE_CTR_TOP_LCD;
	}
	else if(desc.width == 320 && desc.height == 240)
	{
		screensz = SIZE_CTR_BOTTOM_LCD;
	}
	else if(desc.width == 352 && desc.height == 288)
	{
		screensz = SIZE_CIF;
	}
	// auto buffer = (u16*)linearAlloc(bufsz * sizeof(u16) * 2);
	auto cam_buffer = std::make_unique<u16[]>(bufsz * videobuffer.NUM_BUFS);
	const auto buffer = cam_buffer.get();
	auto work_buffer = std::make_unique<unsigned char[]>(bufsz * 3);

	const auto inout = SELECT_IN1_OUT2;
	(CAMU_SetSize(inout, screensz, CONTEXT_A));
	(CAMU_SetOutputFormat(inout, OUTPUT_RGB_565, CONTEXT_A));
	(CAMU_SetFrameRate(inout, FRAME_RATE_20));
	(CAMU_SetNoiseFilter(inout, true));
	(CAMU_SetAutoExposure(inout, true));
	(CAMU_SetAutoWhiteBalance(inout, true));

	(CAMU_SetTrimming(PORT_CAM1, false));
	(CAMU_SetTrimming(PORT_CAM2, false));

	(CAMU_GetMaxBytes(&transferUnit, desc.width, desc.height));
	(CAMU_SetTransferBytes(PORT_BOTH, transferUnit, desc.width, desc.height));
	
	(CAMU_Activate(inout));

	(CAMU_GetBufferErrorInterruptEvent(&camReceiveEvent[1], PORT_CAM1));
	(CAMU_GetBufferErrorInterruptEvent(&camReceiveEvent[2], PORT_CAM2));

	(CAMU_ClearBuffer(PORT_BOTH));
	(CAMU_SynchronizeVsyncTiming(SELECT_IN1, SELECT_OUT2));
	(CAMU_StartCapture(PORT_CAM1));

	int buffer_idx = -1;
	auto handle_pixels = [&](std::span<const u16> px_in) -> void {
		if(csock < 0 || !cam_on || !videobuffer.can_write())
			return;

		std::size_t idx = 0;
		for(const auto px : px_in)
		{
			work_buffer[idx++] = ((px & 0xF800) >> 11) << 3;
			work_buffer[idx++] = ((px & 0x07E0) >> 5) << 2;
			work_buffer[idx++] = (px & 0x001F) << 3;
		}

		int outsz = 0;
		qoi_encode(videobuffer.arr[videobuffer.current_writing], work_buffer.get(), &desc, &outsz);
		videobuffer.next_write_sz(outsz);
	};

	auto active_port = PORT_CAM1;
	int old_front = 0;
	auto start_scan = [&](bool captureInterrupted) {
		const auto ifront = int(using_front);
		if(ifront != std::exchange(old_front, ifront))
		{
			CAMU_StopCapture(active_port);
			CAMU_ClearBuffer(active_port);
			captureInterrupted |= true;
			active_port = (old_front ? PORT_CAM2 : PORT_CAM1);
		}

		if(!captureInterrupted)
			buffer_idx = (buffer_idx + 1) % videobuffer.NUM_BUFS;

		const auto b = buffer + (buffer_idx * bufsz);
		(CAMU_SetReceiving(&camReceiveEvent[3], b, active_port, bufsz * sizeof(u16), (s16)transferUnit));

		if(captureInterrupted)
		{
			(CAMU_StartCapture(active_port));
		}
	};
	start_scan(false);

	bool cancel = false;
	while(!cancel)
	{
		s32 index = -1;
		Result res = svcWaitSynchronizationN(&index, camReceiveEvent, std::size(camReceiveEvent), false, U64_MAX);
		// fprintf(stderr, "event %ld: %08lX\n", index, res);
		if(R_SUCCEEDED(res))
		{
			switch (index) {
			case 0:
				fprintf(stderr, "get cancelled CAM!\n");
				cancel = true;
				break;
			case 1:
				if(active_port == PORT_CAM1)
				{
					svcCloseHandle(camReceiveEvent[3]);
					camReceiveEvent[3] = 0;
					start_scan(true);
				}
				break;
			case 2:
				if(active_port == PORT_CAM2)
				{
					svcCloseHandle(camReceiveEvent[3]);
					camReceiveEvent[3] = 0;
					start_scan(true);
				}
				break;
			case 3:
				svcCloseHandle(camReceiveEvent[3]);
				camReceiveEvent[3] = 0;
				{
				const int idx = buffer_idx;
				start_scan(false);
				handle_pixels(std::span(buffer + (idx * bufsz), bufsz));
				}
				break;
			default:
				fprintf(stderr, "uh oh\n");
				break;
			}
		}
		else break;
	}

	(CAMU_StopCapture(PORT_CAM1));
	(CAMU_ClearBuffer(PORT_CAM1));
	(CAMU_Activate(SELECT_NONE));

	// linearFree(buffer);

	std::for_each(std::next(std::begin(camReceiveEvent)), std::end(camReceiveEvent), [](auto& e) {
		if(e != 0)
		{
			svcCloseHandle(e);
			e = 0;
		}
	});
}

int main(int argc, char **argv)
{
	Result ret = 0;

	gfxInitDefault();
	C3D_Init(C3D_DEFAULT_CMDBUF_SIZE);
	C2D_Init(C2D_DEFAULT_MAX_OBJECTS);
	C2D_Prepare();

	romfsInit();

	consoleDebugInit(debugDevice_SVC);
	osSetSpeedupEnable(true);
	
	// u32 apt_oldval;
	// APT_GetAppCpuTimeLimit(&apt_oldval);
	// APT_SetAppCpuTimeLimit(30);

	C3D_RenderTarget* top = C2D_CreateScreenTarget(GFX_TOP, GFX_LEFT);
	C3D_RenderTarget* bottom = C2D_CreateScreenTarget(GFX_BOTTOM, GFX_LEFT);
	
	SOC_buffer = (u32*)memalign(SOC_ALIGN, SOC_BUFFERSIZE);
	if(SOC_buffer == nullptr)
	{
		snprintf(errbuf, 1024, "memalign: failed to allocate\n");
		show_error();
	}

	// Now intialise soc:u service
	const bool soc_ready = SOC_buffer && R_SUCCEEDED(ret = socInit(SOC_buffer, SOC_BUFFERSIZE));
	if (!soc_ready && SOC_buffer)
	{
		snprintf(errbuf, 1024, "socInit: 0x%08lX\n", ret);
		show_error();
	}

	const bool cam_ready = R_SUCCEEDED(ret = camInit());
	if (!cam_ready)
	{
		snprintf(errbuf, 1024, "camInit: 0x%08lX\n", ret);
		show_error();
	}
	
	MIC_buffer = (u8*)memalign(MIC_ALIGN, MIC_BUFFERSIZE);
	if(MIC_buffer == nullptr)
	{
		snprintf(errbuf, 1024, "memalign: failed to allocate\n");
		show_error();
	}

	const bool mic_ready = MIC_buffer && R_SUCCEEDED(ret = micInit(MIC_buffer, MIC_BUFFERSIZE));
	if (!mic_ready && MIC_buffer)
	{
		snprintf(errbuf, 1024, "micInit: 0x%08lX\n", ret);
		show_error();
	}
	else
	{
		micbuf_datasize = micGetSampleDataSize();
	}

	constexpr u32 clear_color = C2D_Color32(192,192,192,255);
	constexpr u32 text_color = C2D_Color32(224,224,224,255);
	constexpr u32 info_rect_color = C2D_Color32(144,144,144,255);
	auto sheet = C2D_SpriteSheetLoad("romfs:/gfx/sprites.t3x");
	auto buf = C2D_TextBufNew(512);
	auto dynamic_buf = C2D_TextBufNew(128);

	C2D_Text toggle_cam_txt, toggle_mic_txt, ready_conn_txt, exit_conn_txt, exit_app_txt;
	C2D_TextParse(&toggle_cam_txt, buf, "\uE000\nToggle video");
	C2D_TextParse(&toggle_mic_txt, buf, "\uE001\nToggle audio");
	C2D_TextParse(&ready_conn_txt, buf, "\uE002\nStart");
	C2D_TextParse(&exit_conn_txt, buf, "\uE002\nStop");
	C2D_TextParse(&exit_app_txt, buf, "\uE003\nExit");

	C2D_Sprite button_sprites[4];
	#define ADDBUTTON(n, x, y)	C2D_SpriteFromSheet(&button_sprites[n], sheet, sprites_button_red_idx + n); \
								C2D_SpriteSetDepth(&button_sprites[n], 0.0f); \
								C2D_SpriteSetPos(&button_sprites[n], x, y);

	ADDBUTTON(0, 8, 12)
	ADDBUTTON(1, 8 + 160, 12)
	ADDBUTTON(2, 8, 12 + 120)
	ADDBUTTON(3, 8 + 160, 12 + 120)

	#undef ADDBUTTON

	C2D_Sprite instr_spr{};
	C2D_SpriteFromSheet(&instr_spr, sheet, sprites_instructions_idx);
	C2D_SpriteSetDepth(&instr_spr, 1.0f);

	C2D_Sprite top_sprites[8];

	#define ADDSPRITE(n, im, x, y, z)	C2D_SpriteFromSheet(&top_sprites[n], sheet, im); \
										C2D_SpriteSetDepth(&top_sprites[n], z); \
										C2D_SpriteSetPos(&top_sprites[n], x, y);

	ADDSPRITE(0, sprites_webcam_idx, 10, 4, 0)
	ADDSPRITE(1, sprites_microphone_idx, 10, 104, 0)
	ADDSPRITE(2, sprites_computer_idx, 400 - 96 - 24, (204 - 96 - 32)/2, 0)
	ADDSPRITE(3, sprites_connection_idx, (400 - 96) / 2, (204 - 96)/2, 0)

	ADDSPRITE(4, soc_ready ? sprites_valid_idx : sprites_wrong_idx, (400 - 96) / 2, (204 - 96)/2, 1)
	C2D_SpriteSetScale(&top_sprites[4], 0.5f, 0.5f);
	ADDSPRITE(5, cam_ready ? sprites_valid_idx : sprites_wrong_idx, 10, 4, 1)
	C2D_SpriteSetScale(&top_sprites[5], 0.5f, 0.5f);
	ADDSPRITE(6, mic_ready ? sprites_valid_idx : sprites_wrong_idx, 10, 104, 1)
	C2D_SpriteSetScale(&top_sprites[6], 0.5f, 0.5f);
	ADDSPRITE(7, sprites_wrong_idx, 400 - 96 - 24, (204 - 96 - 32)/2, 0)
	C2D_SpriteSetScale(&top_sprites[7], 0.5f, 0.5f);

	C2D_Image info_images[3] = {
		C2D_SpriteSheetGetImage(sheet, sprites_disabled_idx),
		C2D_SpriteSheetGetImage(sheet, sprites_valid_idx),
		C2D_SpriteSheetGetImage(sheet, sprites_wrong_idx),
	};

	bool using_front_camera = false;

	qoi_desc desc{};
	desc.width = 320;
	desc.height = 240;
	desc.channels = 3;
	desc.colorspace = QOI_LINEAR;

	VideoBuffer_t videobuffer;
	const unsigned outpixelsize = qoi_max_size(&desc);
	auto outpixeldata = std::make_unique<unsigned char[]>(outpixelsize * videobuffer.NUM_BUFS);
	auto outpixel_ptr = outpixeldata.get();
	memset(outpixel_ptr, 0, outpixelsize * videobuffer.NUM_BUFS * sizeof(unsigned char));

	for(unsigned i = 0; i < videobuffer.NUM_BUFS; ++i)
	{
		videobuffer.arr[i] = outpixel_ptr;
		videobuffer.sizes[i] = outpixelsize;
		outpixel_ptr += outpixelsize;
	}
	

	AudioBuffer_t audiobuffer;
	const unsigned outsamplesize = (32730/6) * sizeof(s16) + 4;
	auto outsampledata = std::make_unique<u8[]>((outsamplesize + 16) * audiobuffer.NUM_BUFS);
	auto outsample_ptr = outsampledata.get();
	memset(outsample_ptr, 0, outsamplesize * audiobuffer.NUM_BUFS * sizeof(u8));

	for(unsigned i = 0; i < audiobuffer.NUM_BUFS; ++i)
	{
		memcpy(outsample_ptr, "PCMA", 4);
		audiobuffer.arr[i] = outsample_ptr;
		audiobuffer.sizes[i] = outsamplesize;
		outpixel_ptr += outsamplesize + 16;
	}

	Handle event_stop;
	svcCreateEvent(&event_stop, ResetType::RESET_STICKY);

	ctr::mutex sockets_mutex;
	ctr::thread cam_th, mic_th, send_th;
	if(cam_ready && soc_ready)
	{
		auto meta = ctr::thread::basic_meta;
		meta.prio -= 2;
		cam_th = ctr::thread(meta, capture_cam_thread_func, event_stop, std::ref(using_front_camera), std::cref(desc), std::ref(videobuffer));
	}
	if(mic_ready && soc_ready)
	{
		auto meta = ctr::thread::basic_meta;
		meta.prio -= 3;
		mic_th = ctr::thread(meta, mic_thread_func, event_stop, std::ref(audiobuffer));
	}
	if(soc_ready)
	{
		auto meta = ctr::thread::basic_meta;
		meta.prio -= 1;
		send_th = ctr::thread(meta, send_thread_func, event_stop, std::ref(sockets_mutex), std::ref(videobuffer), std::ref(audiobuffer));
	}

	sockaddr_in server{}, client{};
	socklen_t clientlen = sizeof(client);
	server.sin_family = AF_INET;
	server.sin_port = htons(9975);
	server.sin_addr.s_addr = gethostid();

	while(aptMainLoop())
	{
		hidScanInput();
		if(csock < 0 && sock > 0)
		{
			std::unique_lock lk(sockets_mutex);
			csock = accept(sock, (struct sockaddr *) &client, &clientlen);
			if(csock < 0)
			{
				if(errno != EAGAIN)
				{
					snprintf(errbuf, 1024, "accept: %d %s\n", errno, strerror(errno));
					show_error();
				}
			}
			else
			{
				fputs("connected!\n", stderr);
				// set client socket to blocking to simplify sending data back
				fcntl(csock, F_SETFL, fcntl(csock, F_GETFL, 0) & ~O_NONBLOCK);
				send(csock, "meta", 4, 0);
				u16 dat = htons(desc.width);
				send(csock, &dat, sizeof(dat), 0);
				dat = htons(desc.height);
				send(csock, &dat, sizeof(dat), 0);
				dat = htons(32730u);
				send(csock, &dat, sizeof(dat), 0);
			}
		}

		const u32 kDown = hidKeysDown();
		const u32 kHeld = hidKeysHeld();
		if (kDown & KEY_Y)
		{
			break; // break in order to return to hbmenu
		}
		else if(kDown & KEY_X)
		{
			std::unique_lock lk(sockets_mutex);
			if(csock > 0)
			{
				close(csock);
				csock = -1;
			}

			if(sock > 0)
			{
				close(sock);
				sock = -1;
			}
			else
			{
				sock =  socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
				if(sock < 0)
				{
					snprintf(errbuf, 1024, "socket: %d %s\n", errno, strerror(errno));
					show_error();
				}

				int ret = 0;
				if ( (ret = bind (sock, (struct sockaddr *) &server, sizeof (server))) ) {
					close(sock);
					sock = -1;
					snprintf(errbuf, 1024, "bind: %d %s\n", errno, strerror(errno));
					show_error();
				}

				// Set socket non blocking so we can still read input to exit
				fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);

				if ( (ret = listen( sock, 1)) )
				{
					close(sock);
					sock = -1;
					snprintf(errbuf, 1024, "listen: %d %s\n", errno, strerror(errno));
					show_error();
				}
			}
		}
		else if(kDown & KEY_A)
		{
			if(cam_ready)
			{
				cam_on = !cam_on;
				top_sprites[5].image = info_images[cam_on];
			}
		}
		else if(kDown & KEY_B)
		{
			if(mic_ready)
			{
				mic_on = !mic_on;
				top_sprites[6].image = info_images[mic_on];
			}
		}

		if(sock < 0 || csock < 0)
		{
			top_sprites[7].image = info_images[2];
		}
		else
		{
			top_sprites[7].image = info_images[1];
		}

		C3D_FrameBegin(C3D_FRAME_SYNCDRAW);
		C2D_TargetClear(bottom, clear_color);
		C2D_TargetClear(top, clear_color);

		C2D_TextBufClear(dynamic_buf);

		C2D_SceneBegin(top);

		for(const auto& spr : top_sprites)
			C2D_DrawSprite(&spr);

		C2D_DrawRectSolid(0, 206, 0, 400, 34, info_rect_color);
		std::optional<C2D_Text> info_txt;
		if(sock > 0 && csock < 0)
		{
			auto ip_txt = &info_txt.emplace();
			snprintf(errbuf, 1024, "My IP: %s", inet_ntoa((in_addr){.s_addr = gethostid()}));
			C2D_TextParse(ip_txt, dynamic_buf, errbuf);
			C2D_TextOptimize(ip_txt);
			C2D_DrawText(ip_txt, C2D_WithColor | C2D_AtBaseline | C2D_AlignCenter, 200.0, 235.0f, 0.5f, 0.625f, 0.625f, text_color);
		}
		else if(sock > 0 && csock > 0)
		{
			auto ready_txt = &info_txt.emplace();
			snprintf(errbuf, 1024, "Communicating...");
			C2D_TextParse(ready_txt, dynamic_buf, errbuf);
			C2D_TextOptimize(ready_txt);
		}
		else if(sock < 0 && csock < 0)
		{
			auto ready_txt = &info_txt.emplace();
			snprintf(errbuf, 1024, "Waiting...");
			C2D_TextParse(ready_txt, dynamic_buf, errbuf);
			C2D_TextOptimize(ready_txt);
		}

		if(kDown & KEY_DUP)
		{
			using_front_camera = !using_front_camera;
		}

		if(info_txt) C2D_DrawText(&*info_txt, C2D_WithColor | C2D_AtBaseline | C2D_AlignCenter, 200.0, 235.0f, 0.5f, 0.625f, 0.625f, text_color);

		C2D_SceneBegin(bottom);

		C2D_Text* txts[4] = {
			&toggle_cam_txt,
			&toggle_mic_txt,
			(sock > 0 ? &exit_conn_txt : &ready_conn_txt),
			&exit_app_txt
		};

		for(int i = 0; i < 4; ++i)
		{
			float w = 0, h = 0;
			C2D_DrawSprite(&button_sprites[i]);
			C2D_TextGetDimensions(txts[i], 0.75f, 0.75f, &w, &h);
			C2D_DrawText(	txts[i], C2D_WithColor | C2D_AlignCenter,
							button_sprites[i].params.pos.x + (button_sprites[i].params.pos.w) * 0.5f,
							button_sprites[i].params.pos.y + floorf((button_sprites[i].params.pos.h - h) * 0.5f),
							0.5f, 0.75f, 0.75f, text_color);
		}

		if(kHeld & KEY_SELECT)
		{
			C2D_DrawSprite(&instr_spr);
		}

		C3D_FrameEnd(0);
	}

	svcSignalEvent(event_stop);

	std::unique_lock lk(sockets_mutex);
	if(send_th)
	{
		send_th.join();
		if(csock > 0)
		{
			close(csock);
			csock = -1;
		}

		if(sock > 0)
		{
			close(sock);
			sock = -1;
		}
	}
	if(mic_th)
	{
		mic_th.join();
	}
	if(cam_th)
	{
		cam_th.join();
	}

	svcCloseHandle(event_stop);

	C2D_TextBufDelete(buf);
	C2D_SpriteSheetFree(sheet);

	if(mic_ready)
		micExit();
	
	if(cam_ready)
		camExit();

	if(soc_ready)
		socExit();
	
	free(SOC_buffer);
	free(MIC_buffer);

	// APT_SetAppCpuTimeLimit(apt_oldval);

	romfsExit();

	C2D_Fini();
	C3D_Fini();
	gfxExit();

	return 0;
}
