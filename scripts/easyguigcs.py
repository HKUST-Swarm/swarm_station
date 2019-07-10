#!/usr/bin/python
from Tkinter import *
from ttkthemes import ThemedTk

class App:

    def __init__(self, master):

        frame = Frame(master)
        frame.pack()

        self.button = Button(
            frame, text="QUIT", fg="red", command=frame.quit
            )
        self.button.pack(side=LEFT)

        self.hi_there = Button(frame, text="Hello", command=self.say_hi)
        self.hi_there.pack(side=LEFT)

    def say_hi(self):
        ok = Button(dialog)
        print "hi there, everyone!", ok

root = ThemedTk(theme="arc")

app = App(root)

root.mainloop()
root.destroy() # optional; see description below