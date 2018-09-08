import playsound

class IntroSketch:
    def __init__(self):
        pass

    def intro_sketch(self):
        playsound.playsound('util/greeting.mp3', True) #greeting

    def enter_esc(self):
        playsound.playsound('util/enter_esc.mp3', True) #enter or esc

    def select_again(self):
        playsound.playsound('util/select_again.mp3', True) #select again