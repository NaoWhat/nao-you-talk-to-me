__author__ = 'Leandra'
from naoqi import ALProxy
import speech_recognition as sr
import time
import ftplib

from os import path
AUDIO_FILE = path.join(path.dirname(path.realpath(__file__)), "english.wav")

NAO_IP = "lapis.local"
NAO_AUDIO_FILE = "/home/nao/recordings/microphones/test.wav"
LOCAL_AUDIO_FILE = path.join(path.dirname(path.realpath(__file__)), "test.wav")



tts = ALProxy("ALTextToSpeech", NAO_IP , 9559)
audio_recorder = ALProxy("ALAudioRecorder", NAO_IP, 9559)
#motion = ALProxy("ALMotion", NAO_IP , 9559)

#motion.setStiffnesses("Body", 1.0)
#motion.moveInit()
#move_id = motion.moveTo(0.5, 0, 0)
channels = [0, 0, 0, 1] #[left, right, front, rear]
print("starting")
audio_recorder.startMicrophonesRecording(NAO_AUDIO_FILE, "wav", 16000, channels)
#tts.say(str(transcription))
time.sleep(5)
#motion.wait(move_id, 0)
#tts.say("Hello world")
audio_recorder.stopMicrophonesRecording()
print("stopped recording")


#Open ftp connection
ftp = ftplib.FTP(NAO_IP, 'nao',
'admin')
#Get the test.wav file
testFile = open(LOCAL_AUDIO_FILE, "wb")
ftp.retrbinary('RETR recordings/microphones/test.wav', testFile.write)
testFile.close()
ftp.quit()

# use the audio file as the audio source
r = sr.Recognizer()
with sr.AudioFile(LOCAL_AUDIO_FILE) as source:
    audio = r.record(source) # read the entire audio file



# recognize speech using Google Speech Recognition
try:
    # for testing purposes, we're just using the default API key
    # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
    # instead of `r.recognize_google(audio)`
    transcription = r.recognize_google(audio)
    print("Google Speech Recognition thinks you said " + r.recognize_google(audio))
except sr.UnknownValueError:
    print("Google Speech Recognition could not understand audio")
    transcription = "what"
except sr.RequestError as e:
    print("Could not request results from Google Speech Recognition service; {0}".format(e))
    transcription = "no wifi"

#must cast unicode to string
tts.say(str(transcription))