import imgaug as ia
import imgaug.augmenters as iaa
#from imgaug.augmentables.bbs import BoundingBox, BoundingBoxesOnImage
import cv2 

#Inspriration taking from https://github.com/aleju/imgaug
class augmentation:

    def __init__(self):

        self.sed = ia.seed(1)
        self.sometimes = lambda aug: iaa.Sometimes(1.0, aug)
        self.seq = iaa.Sequential(
                [
                
                #Blur each image with varying strength using
                #Gaussian blur (sigma between 0 and 3.0),
                #Average/uniform blur (kernel size between 2x2 and 7x7)
                #Median blur (kernel size between 3x3 and 11x11).
                iaa.OneOf([
                    iaa.Rain(drop_size=(0.8, 0.90)),
                ]),
                
                iaa.SomeOf((0, 5),
                    [
                        #Blur each image with varying strength using
                        #Gaussian blur (sigma between 0 and 3.0),
                        #Average/uniform blur (kernel size between 2x2 and 7x7)
                        #Median blur (kernel size between 3x3 and 11x11).
                        iaa.OneOf([
                            iaa.GaussianBlur((1, 7.0)),
                        ]),
                
                        #Add gaussian noise to some images.
                        #In 50% of these cases, the noise is randomly sampled per
                        #channel and pixel.
                        #In the other 50% of all cases it is sampled once per
                        #pixel (i.e. brightness change).
                        iaa.AdditiveGaussianNoise(
                            loc=0, scale=(0.0, 0.55*255), per_channel=0.5
                        ),

                        #Change brightness of images (50-150% of original value).
                        #iaa.Multiply((0.8, 1.2), per_channel=0.5),

                    ],
                    #Do all of the above augmentations in random order
                    random_order=True
                )
            ],
            random_order=True
        )

if __name__=="__main__":
 
    aug = augmentation()

    image = cv2.imread('aug_data/original_one_pattern_aruco.png')
    
    # Augment BBs and images.
    for i in range(5):
        image_aug = aug.seq(image=image)
        cv2.imwrite('aug_image' + str(i) + '.png',image_aug)
