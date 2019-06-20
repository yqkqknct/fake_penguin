from gtts import gTTS
import os
tts = gTTS(text='I am a fake penguin', lang='en')
tts.save('default.mp3')
