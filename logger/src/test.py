import subprocess


p = subprocess.Popen(['roslaunch','audio_capture','capture_to_file.launch','device:=0'])#,'dst:=%s/audio_log.mp3'%dir])
