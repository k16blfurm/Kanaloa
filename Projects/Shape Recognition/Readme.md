# Deep Learning with MATLAB for shape recognition

For the Kanaloa team project, we were presented with the task of image detecting shapes in real time for the Maritime Robotx competition for various reasons. These reasons include the detection for docking the boat, detecting waypoints and much more. Due to this, we decided to use some type of Neural Network to solve this issue. Our two, almost defacto choices to do this was MATLAB (R2018b to be exact) and TensorFlow with a custom made Neural Network. We chose these two out of the many other deep learning programs because either their customizability (in TensorFlow’s case) or their (sort of) ease of use (like in MATLAB’s case).

Due to the fact that we tried TensorFlow and its results were clunky and hard to use, we decided to go with MATLAB due to its proven track record with embedded systems (sort of like the NUC in this case) and having a working Convolutional Neural Network Toolkit for download for about 4 years already.

# Design Process:
## Prerequisites:
To start, you need to have MATLAB R2018b installed with these software packages/plugins:

![image](https://github.com/k16blfurm/Kanaloa/blob/master/Projects/Shape%20Recognition/pasted%20image%200.png)

Also, to get the USB Webcam working in in MATLAB, you need the Webcam hardware package add-on from MATLAB.
https://www.mathworks.com/help/supportpkg/usbwebcams/ug/acquire-images-from-webcams.html

## Image Labeling:
After that, you can start collecting data, labeling, and training.

First, we need to collect data from the USB camera by using using the built in camera modules in Windows or Linux.

When we take pictures, they are taken in 1920x1080, which needs to be downscaled to a resolution of 960x540 in order to be labelled, trained, and processed by our scripts.
If the images are taken with a camera that is not the logitech USB webcams, the resolution of the camera must be noted and a new image resize script must be made to crop and downscale the images to the correct resolution.  If the images need to be cropped, they must be cropped in order to prevent image distortion when downscaling.  You can search for the correct resize script in the imageResizeScripts folder in the drive, or copy and modify one to resize images from a completely new resolution.  In this case, we would use bulkResize.m script to downscale the 1920x1080 images.  We can resize them using the imresize or imcrop.

For more information on how imcrop works:
https://www.mathworks.com/help/images/ref/imcrop.html
For more information on how imresize works:
https://www.mathworks.com/help/images/ref/imresize.html

We need to collect as much image data as possible in order to maximize the accuracy of the network model, which means that thousands of images must be obtained.  These images must be taken with different backgrounds in different configurations, lighting conditions, depths, and angles. (In our case, we managed to come up with a little over 3000 images from Campus, Coconut Island, and a couple more. 

So in this certain case, we were feeding in a 16:9 file and not cropping it at all, 

![image](https://github.com/k16blfurm/Kanaloa/blob/master/Projects/Shape%20Recognition/Pictures%20for%20Final%20Report/pasted%20image%200%20(1).png)

After you convert your images, you need to go to the Apps tab in Matlab and open the Image Labeler app to begin image labelling or add to our existing image labelling session.

![image](https://github.com/k16blfurm/Kanaloa/blob/master/Projects/Shape%20Recognition/Pictures%20for%20Final%20Report/unnamed.png)

Here is an image from our labelling session.
On the left, you can add your labels and you can bounding box the shapes from here by clicking and dragging a box around the shape of interest.  Make sure to label the shapes correctly. 

Once your images have been labelled, you need to export your labels to your workspace for training.  This will create a groundtruth.m file (or whatever you want, making sure to follow the guides set on the github for file naming) which you will need to save in order to utilize the labelled images with the TrainRCNN script.  This script will train the network.

## Training:

Going from here, we can open the TrainRCNN program to train the neural network.  Note that the training will take a while, especially if you are training a few thousand images.  Using a computer with a GPU is highly recommended to speed up the training process.

Here is a screenshot of our code:

![image](https://github.com/k16blfurm/Kanaloa/blob/master/Projects/Shape%20Recognition/Pictures%20for%20Final%20Report/pasted%20image%200%20(2).png)

This is the section of the code where a training dataset is created from the groundtruth file.  You can also see the image properties.
As we can see above, we can load our labels into the TrainRCNN by using the line of code: groundTruth = load(....).

Also, you can see on the bottom of the image that we can change the Image Properties for what images we are going to send into the network for training.  In our case, we will be using an image resolution of 960x540.

Also, you need to declare your labels “{'circle','cruciform','triangle'}”

From here we can just run the script in MATLAB after this and it will produce a trained RCNN model which we will use to validate the network with. Make sure to save your workspace to save the RCNN since we will be using this later on for validation.

Now, we need to open the TrainingWork.m script to validate our images and test out the network. We first load the RCNN network using this command:

![image](https://github.com/k16blfurm/Kanaloa/blob/master/Projects/Shape%20Recognition/Pictures%20for%20Final%20Report/pasted%20image%200%20(6).png)

We will load in the data with the load command.

## Validation:

Going from here, we can use our code to change our height and width, as shown below.

![image](https://github.com/k16blfurm/Kanaloa/blob/master/Projects/Shape%20Recognition/Pictures%20for%20Final%20Report/pasted%20image%200%20(3).png)

Also, we can load our images by using the imread command or load images from the webcam by referencing the webcam in our code (as shown below).

![image](https://github.com/k16blfurm/Kanaloa/blob/master/Projects/Shape%20Recognition/Pictures%20for%20Final%20Report/unnamed%20(1).png)

Here is the line of code which processes and classifies an image one at a time.  To use this, you would need to load in all of your validation images into your working directory and type in the name of the image you want to test out within the parenthesis in the imread function.  All you would need to do is replace the purple text with the image name.  Then you would run the script.

![image](https://github.com/k16blfurm/Kanaloa/blob/master/Projects/Shape%20Recognition/Pictures%20for%20Final%20Report/unnamed%20(1).png)

Here is the chunk of code that allows the script to work with a live stream of images where the camera automatically takes a picture after every few seconds, resizes the image to the correct resolution, and then classifies the shapes that it sees in the image.  In order to use this portion of the code, you must have a USB webcam plugged into the computer and determine the webcam number called in this line of code:

mycam = webcam(camera number);

You can determine the webcam number by typing this into the command line:

camList = webcamlist

This will print out the camera and camera number that Matlab is seeing.  Usually the Logitech USB webcams will be camera number 2.  The snapshot line in the code is what takes the pictures from the live video stream.  To check if the camera is linked to Matlab and the script, you can type this into the command line:

preview(cam);

This will cause a window to pop up which will display the video that the camera is seeing.  If you are getting any errors while running this script, check to see if Matlab is recognizing the USB webcam and that you have the webcam hardware plugin installed in Matlab.

Here is a link to the documentation for processing images from live video in Matlab:
https://www.mathworks.com/help/supportpkg/usbwebcams/examples/acquiring-a-single-image-in-a-loop.html?prodcode=ML

If you are using a GPU and are getting an error while running the script, this may be caused by the GPU running out of memory.  An example of this error message is shown below:

![image](https://github.com/k16blfurm/Kanaloa/blob/master/Projects/Shape%20Recognition/Pictures%20for%20Final%20Report/pasted%20image%200%20(4).png)

This is what you’ll get if you try to run a lackluster GPU without enough memory to validate the network. For this step, it’s recommended to run with a CPU if your graphics buffer is less than 4gb (or 3.5gb in my case for the GTX 970).

We can also change the threshold of validation as shown below.  Only bounding boxes that meet or exceed the threshold will be displayed to the screen during validation.  If you run the classifier model and the image is being returned to the screen without any bounding boxes, the threshold may be too high which is causing the bounding boxes to be thrown out and ignored.

![image](https://github.com/k16blfurm/Kanaloa/blob/master/Projects/Shape%20Recognition/Pictures%20for%20Final%20Report/Screenshot%20from%202018-12-07%2022-03-22.png)

Shown above is what the script will poop out after the validations are checked.  As you can see, multiple bounding boxes are being printed to the screen which identifies all three shapes that the classifier sees.  Ideally each of the bounding boxes around a shape will be combined into one bounding box of the highest confidence level.  In this case, there should be only three bounding boxes that are displayed which is one for each image that the network sees.


## Built With

* MATLAB 2018b


## Authors

* **Blaine Furman** 
* **Hunter Garrette** 


_____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
Image Classifier Documentation

Link to our multi classifier with color recognition:
https://www.pyimagesearch.com/2018/05/07/multi-label-classification-with-keras/

This site contains the code for the classifier that we are using as well as a thorough explanation of the code from the creator.

Originally:  This classifier categorized clothing and it’s color

Note:  Originally (when we first tested out the classifier with the clothing data) it was thought that the image training data was used because the accuracy was so high; however, this was not the case.  We actually tested it out with clothing images that were not within the training dataset.

To run the classifier (on Windows):
Enter command prompt
Then type: conda activate tfdeeplearning
This lets us run the classifier in Raymond’s environment that he created with his config file.
In order to use Raymond’s environment, follow his instructions on the Kanaloa Github under the Deep Learning Project Directory.
Link:  https://github.com/riplaboratory/Kanaloa/tree/master/Projects/DeepLearning

Execute classifier by running the command:
python train.py --dataset dataset --model fashion.model \
	--labelbin mlb.pickle

In order to run this classifier, you need to install additional libraries that are missing in the anaconda environment that Raymond created with his config file.
These include: keras, imutils, and cv2
To install:  Enter the tfdeeplearning environment using the command given earlier, and type:
pip install keras
pip install imutils
pip install opencv-python

We then replaced the clothing training images with a shape dataset from Kaggle:
https://www.kaggle.com/smeschke/four-shapes
This shape dataset contains 4 images: Circle, Square, Star, and Triangle
We threw each shape into separate folders and named them:
Circle_black
Square_black
Star_black
Triangle_black

This classifier automatically generates labels based on the names of the folders of the image data that we create.  It’s important to name the folders in this format, or else the classifier will not run and will be stuck in the training phase.

Here is a screenshot of retraining the classifier with shapes:


Training will take a while as this will loop 75 times (75 Epochs), although the number of Epochs can be adjusted to be less (although this will significantly reduce accuracy).

Results:
To test out the classifier, enter in this command:
python classify.py --model fashion.model --labelbin mlb.pickle \
	--image examples/example_01.jpg
Replace the “examples/example_01.jpg” with the path to whatever image you want to feed in.
For example, if we want to feed in an image called circle_01.jpg within the “examples” folder, we would use:
	--image examples/circle_01.jpg

Note: If the classifier does not work at first, simply restart the classifier if this happens and it should work again.

Here are some screenshots of the output result of random images that we have fed in (all of which are failures because they use terrible image data):







Here are the results of images that we have fed in from a slightly better dataset of random images that yield better results:













**As of right now, we tested this classifier on things that it wasn’t trained for since we had trained it on a dataset from Kaggle with only solid black squares, circles, triangles, and stars.

Important Note:  This classifier must be trained on to recognize each shape and color.  So, in this case we have only trained it to recognize the color black since that is the color of all of our shapes within this test data set.  However, if we feed it in an image of a shape of a different color, the classifier doesn’t know what to do and it ends up classifying an image as the wrong color and shape.  This most likely is due to the fact that we did not train it to recognize multiple colors, which causes it to become confused.  When the image contains a white background, and a shape and color that the classifier is trained on, it works fine.  However, once an image is fed in with color in the background, the classifier becomes completely wrong and gets confused.  If we use a much better dataset that is more related to the actual images that we will need to classify, the classifier program should perform as expected (although we have yet to prove this since we need better image data).
In the case of the example image dataset with the clothing, each image contained multiple colors and even people with a white background, with the focal point being the clothing and its color.  However, it was accurate.  Our dataset that we used seems too simple given that each image only contains one shape in a solid black color with a blank white background.  This is most likely the reason why the classifier is currently not performing as expected when feeding it in a random image of a colored shape, and why it gets both the color and the shape completely wrong.

Image Data Collection:
Here is how we got image data online through google images (via breaking the google terms of service):
https://github.com/hardikvasa/google-images-download
This Github repository lists the complete instructions to download and scrape images off of google images using the Windows terminal command line.
Important Note:
You need to specify which directory you want to save the pictures to by typing: -o /[whatever directory you want to save the images to] at the end (-o is output directory)
If you don’t do this, finding where the images downloaded to will be a pain.
You can even choose which format to save the images as (i.e. jpg, png, etc.), and specify how many images you want to download (the number right before the -o).
Here is a screenshot of the code to run in terminal to scrape images:

In this screenshot, we scraped images of polar bears, balloons, and beaches.  This will obviously be replaced by the topic of whatever images you want to scrape (in this case, red squares, blue triangles, yellow circles, etc.)

We downloaded about 1000 images of each possible shape and color to use as (poor quality) testing data (for now) until we get ahold of a better and more accurate dataset that is similar to what we need.
This dataset contains about 16000 images which includes:
1000 red circles
1000 red squares
1000 red triangles
1000 red crosses
1000 green circles
1000 green squares
1000 green triangles
1000 green crosses
1000 blue circles
1000 blue squares
1000 blue triangles
1000 blue crosses
1000 yellow circles
1000 yellow squares
1000 yellow triangles
1000 yellow crosses

Note: This dataset is not a well controlled set, as we scraped off these images from google images using the command line to get as much data as possible.  So, this data will not yield accurate results with this classifier, especially since our classifier was not trained on this particular dataset as mentioned earlier.  Once we get ahold of a much better dataset, we will be able to have a much better idea of how this classifier truly performs.

Alternative to data collection:  We are considering using a Raspberry Pie to crawl around the web and collect approximately 10000 images of shapes to make data collection easier for us.  It sounds ridiculous, but it might actually work.

**We have uploaded the classifier code, the model, and the black shapes dataset that we have used to initially retrain the classifier model to the Kanaloa Team drive under the image recognition folder.

**We have also uploaded the classifier code to GitHub
