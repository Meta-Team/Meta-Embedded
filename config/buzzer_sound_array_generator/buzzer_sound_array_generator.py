import json

"""
This program convert Tone.js-friendly JSON into array of '{frequency, duration[ms]}, ' for buzzer.
Each track is separated into different file 'id.c'

[MidiConvert](https://tonejs.github.io/MidiConvert)
[MIDI tuning standard - Wikipedia](https://en.wikipedia.org/wiki/MIDI_tuning_standard)

"""

note_counter = 0  # count to 7 and then print a new line


def write_note(out_file, feq, duration):
    global note_counter
    out_file.write("{%4d, %4d}, " % (feq, duration))
    print("%d, %d" % (feq, silent_time))
    note_counter += 1
    if note_counter >= 7:
        out_file.write("\n")
        note_counter = 0


with open("one.json", "r") as in_file:
    data = json.load(in_file)

last_end_time = 0
duration_factor = 0.9  # every duration is scaled by this factor

for track in data["tracks"]:

    out_file = open("%s.c" % track["id"], "w")

    for note in track["notes"]:

        silent_time = round((float(note["time"]) - last_end_time) * duration_factor * 1000)
        if silent_time > 0:
            write_note(out_file, 0, silent_time)

        write_note(out_file,
                   int((2 ** ((int(note["midi"]) - 69) / 12.0)) * 440),  # frequency formula for MIDI
                   round(float(note["duration"]) * duration_factor * 1000))

        last_end_time = float(note["time"]) + float(note["duration"])

    out_file.close()
