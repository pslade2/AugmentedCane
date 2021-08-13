# generate text which can be saved to sound_files for later use

# if raspberry pi is plugged into hdmi switch the audio to headphone jack with below line in terminal:
# amixer cset numid=3 1
from gtts import gTTS
#from pygame import mixer
import time
import os

cwd = os.getcwd()
save_dir = cwd+'/../sound_files/'
txt_message = "You have reached the GPS destination."#"Hello there. This is a test."
txt_filename = 'gps_done.mp3'

#from mpyg321.mpyg321 import MPyg321Player
#'en' for american english, 'en-uk' for british, 'en-au' for australian
tts = gTTS(txt_message,lang='en-uk',slow=False)
tts.save(save_dir+txt_filename)
#mixer.init()
#mixer.music.load(save_dir+txt_filename)
#mixer.music.play()
#player=MPyg321Player()
#player.play_song('hello.mp3')
#system_cmnd = "mpg321 \'" + txt_filename + "\'"
#print(system_cmnd)
#os.system(system_cmnd) # this command waits until finished before going on
import subprocess
process = subprocess.Popen(['mpg321',save_dir+txt_filename])
time.sleep(0.05)
print("started command")
time.sleep(3)
print("ending")