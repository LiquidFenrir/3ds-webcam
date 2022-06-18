# 3ds-webcam

Use your 3DS as a networked webcam with this homebrew (and a nifty python script)  
  
Uses the `sounddevice`, `qoi`, and `pyvirtualcam` pip packages, which require you to have OBS and VoiceMeeter installed.  
On Windows, at least. If you use any flavour of *nix, you should know enough to make it work.

## Usage

Launch the 3DSX from the Homebrew Menu on your console, set it up how you want (microphone/camera/both)  
Start the connection  
Run the script with your console's IP address as the only command-line argument  
When you want to stop, stop the connection on your console or exit directly.  
  
WARNING: The instructions mention push-to-talk, but it's not implemented yet!

## License

This project is licensed under the MIT license.  
Uses a modified version of [the QOI library](https://github.com/phoboslab/qoi/blob/master/qoi.h), which is under the MIT.  
Uses my `ctr_thread.h` public domain header-only std::thread replacement for libctru.  
Uses various images from wikimedia (or hand made by me) in the `gfx` folder (public domain).
