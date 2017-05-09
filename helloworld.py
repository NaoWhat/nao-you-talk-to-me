from naoqi import ALProxy
# import speech_recognition as sr
import time
import ftplib

from wit import Wit
client = Wit(access_token="YOUR ACCESS TOKEN")

from os import path

current_mode = "Perform Mode"

def act_on_response(resp):
    global current_mode
    intent = resp['entities']['intent']
    spoken_text = resp['_text']

    max_part_confidence = None
    max_part_value = None
    max_direction_confidence = None
    max_direction_value = None

    if 'direction' in resp['entities'].keys():
        direction = resp['entities']['direction']
        max_direction_confidence = 0.0
        for items in direction:
            if items['confidence'] > max_direction_confidence:
                max_direction_confidence = items['confidence']
                max_direction_value = items['value']

    if 'part' in resp['entities'].keys():
        part = resp['entities']['part']
        max_part_confidence = 0.0
        for items in part:
            if items['confidence'] > max_part_confidence:
                max_part_confidence = items['confidence']
                max_part_value = items['value']

    max_intent_confidence = 0.0
    for items in intent:
        if items['confidence'] > max_intent_confidence:
            max_intent_confidence = items['confidence']
            max_intent_value = items['value']

    if max_intent_value == "move":
        if max_part_value == "arm":
            tts.say("You said " + str(resp['_text']) + ". I understood that you want me to " + str(max_intent_value) + " my " + str(max_part_value) + " in the " + str(max_direction_value) + " direction. My learning model reported a confidence of " + str(int(max_intent_confidence * 100)) + "percent for movement, " + str(int(max_part_confidence * 100)) + " percent for part and " + str(int(max_direction_confidence * 100)) + " percent for direction")
        else:
            tts.say("You said " + str(resp['_text']) + ". I understood that you want me to " + str(max_intent_value) + " in the " + str(max_direction_value) + " direction. My learning model reported a confidence of " + str(int(max_intent_confidence * 100)) + "percent for movement and " + str(int(max_direction_confidence * 100)) + " percent for direction")
    elif max_intent_value == "learn":
        if current_mode == "Learn Mode":
            tts.say("I am already in learn mode")
        else:
            current_mode = "Learn Mode"
            tts.say("You said" + str(resp['_text']) + ". I understand that you want me to " + str(max_intent_value) + ". I have switched to my learn mode now.")
            tts.say("My learning model reported a confidence of " + str(int(max_intent_confidence * 100)) + " percent for " +  str(max_intent_value))
    elif max_intent_value == "perform":
        if current_mode == "Perform Mode":
            tts.say("I am already in perform mode")
        else:
            current_mode = "Perform Mode"
            tts.say("You said" + str(resp['_text']) + ". I understand that you want me to " + str(max_intent_value) + ". I have switched to my perform mode now.")
            tts.say("My learning model reported a confidence of " + str(int(max_intent_confidence * 100)) + " percent for " +  str(max_intent_value))

AUDIO_FILE = path.join(path.dirname(path.realpath(__file__)), "english.wav")

NAO_IP = "lapis.local"
NAO_AUDIO_FILE = "/home/nao/recordings/microphones/test.wav"
LOCAL_AUDIO_FILE = path.join(path.dirname(path.realpath(__file__)), "test.wav")
CLIENT_ACCESS_TOKEN = "6178e0ebf1a44a8bbc24d51f1fb31dab"

tts = ALProxy("ALTextToSpeech", NAO_IP , 9559)
audio_recorder = ALProxy("ALAudioRecorder", NAO_IP, 9559)
#motion = ALProxy("ALMotion", NAO_IP , 9559)

#motion.setStiffnesses("Body", 1.0)
#motion.moveInit()
#move_id = motion.moveTo(0.5, 0, 0)
channels = [0, 0, 1, 0] #[left, right, front, rear]

while(True):
    print("Hit any key to begin recording!")
    wait = raw_input()
    audio_recorder.startMicrophonesRecording(NAO_AUDIO_FILE, "wav", 16000, channels)
    #tts.say(str(transcription))
    # time.sleep(5)
    print("Hit any key to stop recording!")
    wait = raw_input()
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
    # r = sr.Recognizer()
    # with sr.AudioFile(LOCAL_AUDIO_FILE) as source:
    #     audio = r.record(source) # read the entire audio file

    resp = None
    with open(LOCAL_AUDIO_FILE, 'rb') as f:
      resp = client.speech(f, None, {'Content-Type': 'audio/wav'})
    print('Yay, got Wit.ai response: ' + str(resp))
    act_on_response(resp)



# # recognize speech using Google Speech Recognition
# try:
#     # for testing purposes, we're just using the default API key
#     # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
#     # instead of `r.recognize_google(audio)`
#     transcription = r.recognize_google(audio)
#     print("Google Speech Recognition thinks you said " + r.recognize_google(audio))
# except sr.UnknownValueError:
#     print("Google Speech Recognition could not understand audio")
#     transcription = "what"
# except sr.RequestError as e:
#     print("Could not request results from Google Speech Recognition service; {0}".format(e))
#     transcription = "no wifi"

#must cast unicode to string
