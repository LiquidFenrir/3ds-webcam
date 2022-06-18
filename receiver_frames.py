import numpy as np
import qoi
import socket
import struct
import sounddevice
# import scipy.signal
import pyvirtualcam
import sys
import traceback

# packsize = int(sys.argv[2])
biggest_yet = bytearray(640 * 480 * 4 + 16)

print("socket!")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("connecting!")
sock.connect((sys.argv[1], 9975))
print("connected!")
try:
	dat = sock.recv(4)
	assert(bytes(dat) == b"meta")
	dat = sock.recv(2)
	scr_w = struct.unpack("!H", dat)[0]
	dat = sock.recv(2)
	scr_h = struct.unpack("!H", dat)[0]
	dat = sock.recv(2)
	aud_sr = struct.unpack("!H", dat)[0]
except Exception as e:
	print("setup exc:", e)
	sock.shutdown(0)
	sock.close()
	exit(1)

# blksz = 32730//15
blksz = 3072
previous_audio = b""
with sounddevice.OutputStream(samplerate=32728, channels=2,blocksize=blksz, device="VoiceMeeter Input (VB-Audio Voi, MME") as ros, pyvirtualcam.Camera(width=scr_w, height=scr_h, fps=20, device="OBS Virtual Camera", backend='obs') as cam:
	try:
		dt = np.dtype('<i2')
		while True:
			# print("receiving!")
			dat = sock.recv(4)
			if len(dat) != 4:
				print(dat)
				print("errored")
				break
			datalen = struct.unpack("!L", dat)[0]
			# print(datalen, "bytes total (", dat, ")")
			if datalen > len(biggest_yet):
				biggest_yet = bytearray(datalen)

			data = biggest_yet
			idx = 0
			yoink = 0
			while idx != datalen:
				dat = sock.recv(datalen - idx)
				l = len(dat)
				# print(f"loop {yoink}: {l}")
				data[idx:idx+l] = dat
				idx += l
				yoink += 1

			# quot, rem = divmod(datalen, packsize)
			# print("have to receive ", quot, "packets and", rem, "extra bytes")
			# for i in range(quot):
			# 	dat = sock.recv(packsize)
			# 	# print(dat)
			# 	if len(dat) != packsize:
			# 		raise Exception(f"problem loop: {len(dat)} vs {packsize} at loop {idx // packsize}")

			# 	data[idx:idx+packsize] = dat
			# 	idx += packsize
			# if rem:
			# 	dat = sock.recv(rem)
			# 	# print(dat)
			# 	if len(dat) != rem:
			# 		raise Exception(f"problem rem: {len(dat)} vs {rem}")
			# 	data[idx:idx+rem] = dat

			# print("done receiving packets")
			head = bytes(data[:4])
			# print("early bytes:", head)
			if head == b"PCMA":
				samps = np.frombuffer(previous_audio + data[4:datalen], dtype=dt) / 32768
				# print(f"audio, {samps.shape} samples ({samps.shape[0]/blksz} blocks)")
				# number_of_samples = round(len(samps) * float(48000) / 32730)
				l = samps.shape[0]
				# resamped = scipy.signal.resample(samps, number_of_samples).astype(dtype=np.float32)
				q, r = divmod(l, blksz)
				for i in range(q):
					idx = i * blksz
					tow = samps[idx:idx+blksz]
					tow = np.column_stack((tow, np.zeros_like(tow)))
					ros.write(tow.astype(dtype=np.float32))

				if r:
					previous_audio = data[(datalen - (r * 2)):datalen]
				else:
					previous_audio = b""

			elif head == b"qoif":
				# print("image")
				cam.send(qoi.decode(bytes(data[:datalen])))
	except Exception as e:
		print("runtime exc:", e)
		print(traceback.format_exc())
	except KeyboardInterrupt:
		print("stopping")

	sock.shutdown(0)
	sock.close()

print("Finish")
