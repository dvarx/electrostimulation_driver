# Interval Cycler Serial Interface

| Command  | Example       | Meaning                                                      |
| -------- | ------------- | ------------------------------------------------------------ |
| `!dbg`   | `!dbg`        | Starts pushing current into the coil.                        |
| `!stp`   | `!stp`        | Stops current from flowing in the coil.                      |
| `!onoff` | `!onoff5000`  | Sets both the Ton and Toff interval to 5000ms.               |
| `!cyc`   | `!cyc720`     | Sets the total number of cycles (#Ton + #Toff) after which the system automatically stops. |
| `!frq`   | `!frq2635000` | Sets the oscillation frequency in millihertz. In this example, oscillation frequency is set to 2'635Hz. |

## Example

`!onoff5000` sets the Ton and Toff interval to 5000ms. `!cyc720` will let the system run for 720 cycles. The total time from start to finish is therefore 720*5s=3600s=1hour.

