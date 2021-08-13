# Quick script to initialize audio files
from gtts import gTTS
import sounddevice
import vlc
import time

#make the text string from the goal value
def build_string(goal):
    if goal == 0:
        return "Return to center."
    out = "Turn " + str(abs(goal))
    out = out + " degrees to the "
    if goal < 0:
        out = out + "left."
    else:
        out = out + "right."
    return out

def make_mp3_from_string(text_string, fname):
    tts = gTTS(text_string, lang='en-uk', slow=False)
    tts.save(fname)

def make_mp3_goals():
    goals = list(range(-90, 100, 10))
    for goal in goals:
        text_string = build_string(goal)
        fname = str(goal) + ".mp3"
        make_mp3_from_string(text_string)

def play_audio(fname):
    p = vlc.MediaPlayer(fname)
    p.play()
    
def check_sound_devices():
    devs = sounddevice.query_devices()
    print(devs) # Shows current output and input as well with "<" abd ">" tokens

def test_wait_for_finish():
    
    p = vlc.MediaPlayer("turn_audio_files/90.mp3")
    p.play()
    time.sleep(0.5)
    while p.get_state() == vlc.State.Playing:
        pass
    time.sleep(0.5)
    p = vlc.MediaPlayer("turn_audio_files/-30.mp3")
    p.play()
    
def main():
    test_wait_for_finish()

if __name__ == "__main__":
    main()