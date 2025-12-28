#!/usr/bin/env python3

#         Python Stream Deck Library
#      Released under the MIT license
#
#   dean [at] fourwalledcubicle [dot] com
#         www.fourwalledcubicle.com
#

# Example script showing basic library usage - updating key images with new
# tiles generated at runtime, and responding to button state change events.

import os
import threading
import random

from PIL import Image, ImageDraw, ImageFont
from StreamDeck.DeviceManager import DeviceManager
from StreamDeck.ImageHelpers import PILHelper
from StreamDeck.Transport.Transport import TransportError

# Folder location of image assets used by this example.
ASSETS_PATH = os.path.join(os.path.dirname(__file__), "Assets")
global procA
procA=0
# 0 or 1
button_state=[0,0,0,0,0,0,0,0 ]


# Generates a custom tile with run-time generated text and custom image via the
# PIL module.
def render_key_image(deck, icon_filename, font_filename, label_text):
    # Resize the source image asset to best-fit the dimensions of a single key,
    # leaving a margin at the bottom so that we can draw the key title
    # afterwards.
    icon = Image.open(icon_filename)
    image = PILHelper.create_scaled_key_image(deck, icon, margins=[0, 0, 20, 0])

    # Load a custom TrueType font and use it to overlay the key index, draw key
    # label onto the image a few pixels from the bottom of the key.
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype(font_filename, 14)
    draw.text((image.width / 2, image.height - 5), text=label_text, font=font, anchor="ms", fill="white")

    return PILHelper.to_native_key_format(deck, image)


# Generate an image for the screen
def render_screen_image(deck, font_filename, text):
    image = PILHelper.create_screen_image(deck)
    # Load a custom TrueType font and use it to create an image
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype(font_filename, 20)
    draw.text((image.width / 2, image.height - 25), text=text, font=font, anchor="ms", fill="white")

    return PILHelper.to_native_screen_format(deck, image)


# Returns styling information for a key based on its position and state.
def get_key_style(deck, key, state):
    # Last button in the example application is the exit button.
    exit_key_index = deck.key_count() - 1

    if key == exit_key_index:
        name = "exit"
        icon = "{}.png".format("Exit")
        font = "Roboto-Regular.ttf"
        label = "Bye" if state else "Exit"
    elif key == 0:
        name = "No.0"
        icon = "{}.png".format("init-pos" if state else "init-pos")
        font = "Roboto-Regular.ttf"
        label = "Init POS" if state else "Init POS"
    elif key == 1:
        name = "No.1"
        icon = "{}.png".format("hello" if state else "hello")
        font = "Roboto-Regular.ttf"
        label = "Hello!" if state else "Hello!"
    elif key == 2:
        name = "No.2"
        icon = "{}.png".format("battery" if state else "battery")
        font = "Roboto-Regular.ttf"
        label = "Battery" if state else "Battery"     
    elif key == 3:
        if button_state[key]==0:
            name = "No.3"
            icon = "{}.png".format("microphone-off" if state else "microphone-on")
            font = "Roboto-Regular.ttf"
            label = "mic ON" if state else "mic ON"
            button_state[key]=1 
        else:
            name = "No.3"
            icon = "{}.png".format("microphone-on" if state else "microphone-off")
            font = "Roboto-Regular.ttf"
            label = "mic OFF" if state else "mic OFF"
            button_state[key]=0
    elif key == 4:
        name = "No.4"
        icon = "{}.png".format("mic-" if state else "mic-")
        font = "Roboto-Regular.ttf"
        label = "mic-" if state else "mic-"
    elif key == 5:
        if button_state[key]==0:
            name = "No.5"
            icon = "{}.png".format("microphone-off" if state else "microphone-on")
            font = "Roboto-Regular.ttf"
            label = "mic ON" if state else "mic ON"
            button_state[key]=1 
        else:
            name = "No.5"
            icon = "{}.png".format("microphone-on" if state else "microphone-off")
            font = "Roboto-Regular.ttf"
            label = "mic OFF" if state else "mic OFF"
            button_state[key]=0
    elif key == 6:
        name = "No.6"
        icon = "{}.png".format("mic+" if state else "mic+")
        font = "Roboto-Regular.ttf"
        label = "mic+" if state else "mic+"      
    elif key == 7:
        name = "No.7"
        icon = "{}.png".format("pcvol" if state else "pcvol")
        font = "Roboto-Regular.ttf"
        label = "Battery" if state else "Battery"         
    else:
        name = "emoji"
        icon = "{}.png".format("Pressed" if state else "Released")
        font = "Roboto-Regular.ttf"
        label = "Pressed!" if state else "Key {}".format(key)

    return {
        "name": name,
        "icon": os.path.join(ASSETS_PATH, icon),
        "font": os.path.join(ASSETS_PATH, font),
        "label": label
    }


# Creates a new key image based on the key index, style and current key state
# and updates the image on the StreamDeck.
def update_key_image(deck, key, state):
    # Determine what icon and label to use on the generated key.
    key_style = get_key_style(deck, key, state)

    # Generate the custom key with the requested image and label.
    image = render_key_image(deck, key_style["icon"], key_style["font"], key_style["label"])

    # Use a scoped-with on the deck to ensure we're the only thread using it
    # right now.
    with deck:
        # Update requested key with the generated image.
        deck.set_key_image(key, image)


# Prints key state change information, updates the key image and performs any
# associated actions when a key is pressed.
def key_change_callback(deck, key, state):
    # Print new key state
    print("Deck {} Key {} = {}".format(deck.id(), key, state), flush=True)

    # Don't try to set an image for touch buttons but set a random color
    if key >= deck.key_count():
        set_random_touch_color(deck, key)
        return

    # Update the key image based on the new key state.
    update_key_image(deck, key, state)

    # Check if the key is changing to the pressed state.
    if state:
        key_style = get_key_style(deck, key, state)
        if key_style["name"] == "No.0":
            print("set Intial Position : 0,0,0")
            proc=subprocess.run(['rosrun','okd_hsrsample','initpose.py']) 
        if key_style["name"] == "No.1":
            print("HSR test : say hello")
            proc=subprocess.run(['rostopic','pub','-1','/talk_request','tmc_msgs/Voice','0','0','0','"みなさんこんにちは、私の名前はHSRです"']) 

        if key_style["name"] == "No.2":
            print("HSR : check battery level")
            proc=subprocess.run(['rosrun','okd_hsrsample','checkbattery.py']) 
        if key_style["name"] == "No.3":
            print("HSR : check mic level")
            if button_state[key]!=0:
                procA=subprocess.Popen(['python3','mic-monitor.py']) 
            #else:
            #    procA.kill()

        if key_style["name"] == "No.4":
            subprocess.run(['amixer', '-D', 'hw:0', 'sset', 'Capture','3%-'])
        if key_style["name"] == "No.5":
            if button_state[key]==0:
                subprocess.run(['amixer', '-D', 'hw:0', 'sset', 'Capture','100'])
            else:
                subprocess.run(['amixer', '-D', 'hw:0', 'sset', 'Capture','0'])   
        if key_style["name"] == "No.6":
            subprocess.run(['amixer', '-D', 'hw:0', 'sset', 'Capture','3%+'])



        # When an exit button is pressed, close the application.
        if key_style["name"] == "exit":
            # Use a scoped-with on the deck to ensure we're the only thread
            # using it right now.
            with deck:
                # Reset deck, clearing all button images.
                deck.reset()

                # Close deck handle, terminating internal worker threads.
                deck.close()


# Set a random color for the specified key
def set_random_touch_color(deck, key):
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)

    deck.set_key_color(key, r, g, b)


if __name__ == "__main__":
    streamdecks = DeviceManager().enumerate()

    print("Found {} Stream Deck(s).\n".format(len(streamdecks)))

    for index, deck in enumerate(streamdecks):
        # This example only works with devices that have screens.
        if not deck.is_visual():
            continue

        deck.open()
        deck.reset()

        print("Opened '{}' device (serial number: '{}', fw: '{}')".format(
            deck.deck_type(), deck.get_serial_number(), deck.get_firmware_version()
        ))

        # Set initial screen brightness to 30%.
        deck.set_brightness(30)

        # Set initial key images.
        for key in range(deck.key_count()):
            update_key_image(deck, key, False)

        # Register callback function for when a key state changes.
        deck.set_key_callback(key_change_callback)

        # Set a screen image
        image = render_screen_image(deck, os.path.join(ASSETS_PATH, "Roboto-Regular.ttf"), "Python StreamDeck")
        deck.set_screen_image(image)

        # Wait until all application threads have terminated (for this example,
        # this is when all deck handles are closed).
        for t in threading.enumerate():
            try:
                t.join()
            except (TransportError, RuntimeError):
                pass
