# Computer vision for 'Dobble' game

Idea : Support player with computer vision system to recognize and compare objects on the cards

<img src="https://github.com/vfdev-5/DoubleGameVision/blob/master/Data/Test/jeudobble.jpg" width="250"/>


### Algorithm (08/09/2015) :
    1) Detect cards on the image
        a) Simplify image (i.g. low-band pass filtering, median blur, etc)
        b) Derivate -> Threhold -> Morpho -> Find contours
        c) Select contours by size and length/surface ratio
    2) 'Unify' size of detected cards
        a) Resize all detected cards to a unique size
    3) Extract and match objects between any two cards
        a) Loop on cards taking two different cards
        b) For each card extract objects
            1) Median blur -> Canny -> Morpho -> Find contours
            2) Select contours :
               a) bounding rect of the contour larger min area and smaller 1/4 of card size
               b) distance between center of the contour and the card center is smaller than card radius
               c) max dimension of contour is smaller than card radius
               d) contour brect should not touch (+/- 1 pixel) image boundaries
        c) Extract keypoints and descriptors for the reference object of the card one
            1) Feature extraction uses AKAZE algorithm: http://docs.opencv.org/master/d8/d30/classcv_1_1AKAZE.html
                Pablo F Alcantarilla, Jesús Nuevo, and Adrien Bartoli. Fast explicit diffusion for accelerated
                features in nonlinear scale spaces. Trans. Pattern Anal. Machine Intell, 34(7):1281–1298, 2011.
        d) Extract keypoints and descriptors for a test object of the card two
        e) Match keypoints and select matched keypoints
        f) Decide whether objects match and continue the loop


Problems :

    1) Card contours are too weak -> no card detection
        - weak card contours, strong small contours
        - weak card contours are neglected on threshold

        => Use a generic method to extract cards or objects

