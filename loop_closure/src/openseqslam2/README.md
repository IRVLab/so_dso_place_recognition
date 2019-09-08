# OpenSeqSLAM2.0 Toolbox

The second release of OpenSeqSLAM, repackaged as an easy to use and configure toolbox for MATLAB. To get started, simply run "OpenSeqSLAM2()" with no arguments.

## Known Bugs (roughly ordered by severity):

* MATLAB's VideoReader is dodgy when first getting to work on Linux (install gstreamer-plugins-base, link against system libstdc++, etc. seem to help... but not all the time...)
* Under Ubuntu, the window manager sometimes incorrectly places the window title bar over the top of the figure, instead of on top (only current fix is to manually resize window or close and reopen... not sure who is being stupid here Ubuntu Unity or MATLAB - I suspect Unity)
* Closing the Tweak popup resets the selected match in Results GUI (this should only be done when applying a tweak, not closing)
* In PR it is assumed that every query image has a reference image ground truth. Is this a valid assumption???
* Long paths come up ugly in a number of places (add in some sort of 'smart path trimming'...)
* Crash has been observed when closing help during Progress GUI (have not been able to reproduce...)


## TODO List (roughly ordered by value):

* Properly add different image resize methods for image preprocessing
* Make the outlier fading in the Results GUI update in real time (add a listener to the slider value)
* Make the "focus" area size in the Results GUI relative to datset size instead of being an absolute value
* Disable match selection in Results GUI when a popup is open (i.e. enforce ad hoc modal structure)
* The help documentation needs work (check for spelling, grammar, etc.)
* Should probably change code flow so the start GUI window is returned to after a set of results (and a progress GUI can be exited while running)


## Potential Future Feature List (may or may not eventuate...):

* Use Git LFS to manage storage of the samples archive
* Make resizable
* Fix axis management (currently a random mess full of unnecessary calls)
* Clean up of messy code areas (break GUI creation and sizing functions into manageable sub-functions, etc.)
