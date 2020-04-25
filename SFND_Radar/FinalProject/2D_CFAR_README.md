# 2D CFAR Implementation README

In this small report, an overview of how the 2D variant of the Constant False Alarm Rate (CFAR) was implemented as well as the rationale behind the choices made for the hyperparameters and how the edges of the joint Doppler frequency and range spectrum were handled.

## Implementation Steps

As illuminated in the walkthrough video of this section of the course, we iterate over each spatial location in the joint Doppler frequency and range spectrum and apply a variant of the original 1D CFAR technique covered in the course. For brevity, we refer to this spectrum as the Range-Doppler Map or RDM henceforth. For each spatial location in the RDM, we surround a 2D window around this spatial location which thus serves as the centre of this window. There are two sets of hyperparameters we need to consider before moving forward. The number of _Guard Range Rows_ or `Gr` which are the number of rows away from the centre location that are not considered as part of the calculation. Therefore, the total number of Guard Rows within the 2D window of interest is `2 * Gr + 1` with the addition of `1` to account for the centre location of the window. We similarly have the concept of _Guard Doppler Columns_ or `Gd` which are the number of columns away from the centre that are not considered as part of the calculation. The total number of Guard Range Columns within the 2D window of interest is `2 * Gd + 1` . We finally have _Training Range Rows_ and _Training Doppler Columns_ which define the size of the 2D window to examine the elements in the RDM that surround each spatial location in it. These are defined as `Tr` and `Td` . These operate spatially in the same was as the Guard Range Rows and Guard Doppler Columns, thus producing `2 * Tr + 1` and `2 * Td + 1` rows and columns to examine outside of the Guard elements. Therefore, the size of the window under examination is thus `2 * Tr + 2 * Gr + 1` x `2 * Td + 2 * Gd + 1` for the rows and columns of the spatial window respectively.

For each 2D window under examination, we simply average the elements that are considered training elements and ignore the elements that are within the guard window. We then take the average of these elements, then offset this by a known SNR threshold that is the expected Signal-To-Noise ratio of the environment. If the SNR threshold is larger than the average taken by the elements, we set the output of the spatial location which is the centre of the examination window to be 1, else 0. An important nuance is that the RDM was converted into dB, so we need to transform the map into unitless magnitude, find the average then convert back to dB in order add the SNR offset and check the threshold. Another nuance is that we handle the border elements of this RDM where spatial windows that do not fit nicely when collecting elements in the RDM for analysis are ignored such that any spatial locations whose windows are not fully contained within the RDM are set to 0.

Originally in the walkthrough video, two `for` loops were used to navigate over each location in the RDM with another two `for` loops collecting elements within the spatial window, only accumulating those elements that are not within the guard region of the window.  MATLAB `for` loops can be slow as it's an interpreted language, so vectorisation should be used whenever possible.  This 2D summation operation per spatial window can be achieved by [2D convolution](http://www.songho.ca/dsp/convolution/convolution2d_example.html) with an appropriate designed mask.  This mask is simply composed of all `1`s that are of size `2 * Tr + 2 * Gr + 1` x `2 * Td + 2 * Gd + 1` except for the guard region and centre location with are set to 0.  The mask is then normalised by the sum of the components to facilitate averaging per spatial location.  We first transform the RDM from dB into unitless magnitude, perform the convolution on the RDM then convert the result back into dB, and apply the offset to finally get the thresholds per location in the RDM.  In a vectorised fashion, we use logical operators to simultaneously check all elements in this threshold output and see if the corresponding position in the RDM is greater or lesser and set the output value to 0 or 1 accordingly.

## Selection of Hyperparameters

Due to repeated experimentation and observing the output, the following hyperparameters were chosen:

* `Td = 10`
* `Tr = 8`
* `Gr = 4`
* `Gd = 4`
* `offset = 6`

These were primarily chosen because of the walkthrough video, but also because of repeated experimentation to see how the performance varies when we slowly diverge away from the initial hyperparameters chosen.  The initial hyperparameters chosen were already good enough, so they remain in the final version of this project.

## Dealing with the edge cases

Finally, in a vectorised manner we simply use indexing to suppress all of the edges of the output by simply examining how much the halfway point of the rows and columns would be in the proposed mask and setting those locations in the output mask to 0 accordingly.  By using indexing, we leave the remaining elements intact.