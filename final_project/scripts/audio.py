import pyaudio
import speech_recognition as sr

# http://stackoverflow.com/questions/25394329/python-voice-recognition-library-always-listen

def mainfunction():
	r = sr.Recognizer()
	with sr.Microphone() as source:
		audio = r.listen(source)

	user = r.recognize(audio)
	print(user)

while 1:
        mainfunction()