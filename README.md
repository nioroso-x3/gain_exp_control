# gain_exp_control
Small brightness PID control loop for UVC cameras with no automatic gain control when setting a fixed exposure value.

Python and C++ examples. I tested them using a downsampled 5 fps stream, if you use another framerate you might have to change the PID parameters to avoid oscillations.
Use mediamtx or simply gstreamer tees to send the stream to this tool as a gstreamer pipeline and to your endpoint at the same time.

On my use case I set 7 possible exposures that are acceptable. You might alter them to your case.
