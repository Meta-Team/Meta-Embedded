# Use Buzzer

## Initiate
skd_prio is the thread priority for music playing thread.
```c++
BuzzerSKD::init(skd_prio);
```

## Included functions


### BuzzerSKD::play_sound(sound[])

```DOXYGEN
    /**
     * @param sound  An array of note_with_time_t, ending with {Finish(-1), *}
     * @param prio   Priority of the buzzer thread
     */
```
Play the music hard coded in `BuzzerSKD`. For available sounds, just type `BuzzerSKD::sound_***`, a list will show.


### BuzzerSKD::alert_on()

```DOXYGEN
    /**
     * Enable continuous alert
     */
```
Stop the playing music (if any) and start alerting.

### BuzzerSKD::alert_off()

```DOXYGEN
    /**
     * Disable continuous alert
     */
```
Stop alerting and resume music playing (if any).

### BuzzerSKD::alerting()

```DOXYGEN
    /**
     * Return the status of continuous alert
     * @return The status of continuous alert
     */
```
Returns the buzzer status, `true` for alerting.