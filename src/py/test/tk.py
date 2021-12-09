#!/usr/bin/python3

import tkinter as tk
import threading
from PIL import Image, ImageTk

def cb(window):
	global running

	if not running:
		window.destroy()
	else:
		window.after(100, cb, window)

def thread_run():
	global running

	image1 = Image.open("/usr/share/help/id/gnome-help/figures/ubuntu-logo.png")

	window = tk.Tk()

	window.title("Welcome to LikeGeeks app")

	test = ImageTk.PhotoImage(image1)

	lbl = tk.Label(window, image=test)
	lbl.grid(column=0, row=0)

	window.after(100, cb, window)

	window.mainloop()

def start():
	global running
	global preview_thread

	running = True

	preview_thread = threading.Thread(target=thread_run, args=[])
	preview_thread.start()

def stop():
	global running
	global preview_thread

	running = False

	preview_thread.join()
