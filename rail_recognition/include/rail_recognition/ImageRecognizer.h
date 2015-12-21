//
// Created by bhetherman on 3/30/15.
//

#ifndef PROJECT_IMAGE_RECOGNIZER_H
#define PROJECT_IMAGE_RECOGNIZER_H

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <time.h>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <fstream>
#include <string>
#include <dirent.h>
#include <std_srvs/Empty.h>
#include <sys/stat.h>
#include <map>

#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <sensor_msgs/Image.h>

//OpenCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/ml/ml.hpp>

//CPP
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

/*!
 * \class ImageRecognizer
 * \brief provides a framework to train and test an ANN for image recognition
 *
 * ImageRecognizer creates a ROS node that provides methods for loading, training and testing
 * a neural network using OpenCV's built in CvANN_MLP class and multiple features
 * that are also implemented in OpenCV
 */
class ImageRecognizer
{
public:
  /**
   * \brief Constructor
   */
  ImageRecognizer();

  /*!
   * \brief Tests image recognition with the images in the test directory
   */
  void testImageRecognizer();

  void calculateAndSaveFeatures();

  /*!
   * \brief Trains the BOWs and the ANN using the images in the images directory
   */
  void trainImageRecognizer();

  /*!
   * \brief Loads a saved set of BOWs and ANN for image recognition
   */
  void loadImageRecognizer();

  /*!
  * \brief Recognize a given segmented object.
  * \param object an unrecognized segmented object
  * \param recognitionResults vector in which to store the result candidates in descending order of confidence
  */
  void recognizeObject(const rail_manipulation_msgs::SegmentedObject object, std::vector<std::pair< float, std::string> > &recognitionResults);

private:

  /*!
   * \brief Callback for the new segmented objects list.
   *
   * Performs recognition on each of the images in the list and will save them to the specified
   * directory when the save option is set.
   *
   * \param msg The published segmented objects list being received.
   * \return None
   */
  void objectListCallBack(const rail_manipulation_msgs::SegmentedObjectList msg);

  /*!
   * \brief Detects and extracts keypoints and a descriptor
   *
   * \param input The gray image to use for destection
   * \param keypoints A pointer to a keypoint vector to store the found keypoints.
   * \param extractor A pointer to the extractor to use.
   * \param detector A pointer to the detector to use.
   * \return None
   */
  void detectExtractFeatures(cv::Mat &input, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptor,
                             cv::Ptr<cv::DescriptorExtractor> extractor, cv::Ptr<cv::FeatureDetector> detector);

  /*!
   * \brief Detects, extracts and computes the feature vector to be used in the ANN
   *
   * Performs recognition on each of the images in the list and will save them to the specified
   * directory when the save option is set.
   *
   * \param colorInput The RGB_8 color image that we want the feature vector for
   * \param response_hist A pointer to a Mat object to store the feature vector of the image.
   * \return None
   */
  void detectExtractComputeFeatures(cv::Mat colorInput, cv::Mat &response_hist);

  /*!
   * \brief Gets the image files and classes from the images directory
   *
   * Loops throught the images directory finding all the different class folders and then all
   * the files for each class recoding both.
   *
   * \param classes_names A pointer to a location to store all the class names for each file.
   * \param FileNames A pointer to a location to store the File paths for all the class images.
   * \return None
   */
  void getFilesAndClasses(std::vector<std::string> &classes_names, std::vector<std::string> &FileNames);

  /*!
   * \brief Gets the image files and classes from the images directory
   *
   * Loops throught the images directory finding all the different class folders and then all
   * the files for each class recoding both.
   *
   * \param featuresUnclustered A pointer to a Mat object to store the the features to train the BOW with in each row
   * \param extractor A pointer to the DescriptorExtractor to be used to calculate the descripto of each file
   * \param detector A pointer to the FeatureDetector to be used to detect any features in each file.
   * \param FileNames A pointer to a location containing the File paths for all the class images.
   * \return None
   */
  void getFeatures(cv::Mat &featuresUnclustered, cv::Ptr<cv::DescriptorExtractor> extractor,
                   cv::Ptr<cv::FeatureDetector> detector, std::vector<std::string> &FileNames);

  /*!
   * \brief Train the vocabulary of a BOW for a given feature set
   *
   * Trains a BOW based on the given matrix of features and then saves that trained bag file. 
   *
   * \param featureName A string with the name of the feature bag being trained
   * \param featuresUnclustered A Mat object containing the the features to train the BOW with in each row
   * \param vocabulary A Mat object containing the vocabulary fot the trained BOW
   * \return None
   */
  void trainVocabulary(std::string featureName, cv::Mat featuresUnclustered, cv::Mat &vocabulary);

  /*!
   * \brief Calculates the RGB and HSV values. Places results in Mat 
   *
   * Calculates the RGB, hue, saturation values for the provided color image and builds a feature 
   * vector in a Mat object which is part of the input for an ANN.
   *
   * \param colorInput A color image for which to calculate RGB and HSV features
   * \param result_hist A pointer to a Mat object to store the calculate features
   * \return None
   */
  void colorAndHSVDescriptors(cv::Mat colorInput, cv::Mat &result_hist);

  /*!
   * \brief Trains the ANN
   *
   * Trains the ANN using the given training data and number of classes
   *
   * \param inputData An example training sample (probably not needed)
   * \param classesTrainingData A map containing the features vector for all images of each class.
   * \param numClasses An int specifying how many classes there are.
   * \return None
   */
  void trainNN(cv::Mat &inputData, std::map<std::string,cv::Mat> &classesTrainingData, int numClasses);

  /*!
   * \brief Attempts to recognize a given image
   *
   * Tests the given image by calculating its feature vector and then using the ANN to predict.
   *
   * \param colorInput The color image that is to be tested
   * \param testResults A pointer to a vector in which to store the prediction results
   * \return None
   */
  void testImage(cv::Mat colorInput, std::vector<std::pair<float, std::string> > &testResults);

  void normalizeFeatureVector(cv::Mat &featureVector, int numPixels);

  ros::NodeHandle node, pnh; /*!< a handle for this ROS node */

  ros::Subscriber sub; /*!< subscriber handle for the segmented objects list topic */

  cv::Ptr<cv::BOWImgDescriptorExtractor> siftBowide;/*!< BOWImgDescriptorExtractor ptrs for sift bags */
  cv::Ptr<cv::DescriptorMatcher> bfMatcher, flannMatcher;/*!< DescriptorMatcher ptrs for brute force and flann matchers */
  cv::Ptr<cv::FeatureDetector> siftDetector;/*!< FeatureDetector ptrs for sift */
  cv::Ptr<cv::DescriptorExtractor> siftExtractor;/*!< DescriptorExtractor ptrs for sift */
  cv::Ptr<CvANN_MLP> classifier;/*!< CvANN_MLP ptr for the ANN classifier */

  std::vector<std::string> classLegend;/*!< vector of object class names that are in the same order as the ANN */

  int numSiftFeatures;/*!< number of sift features to find */
  int dictionarySize;/*!< the numbers of words in each BOW */
  int histSize;/*!< number of groups in RGB/HSV histograms */

  int numResponses;/*!< number of responses to return */

  bool saveNewImages;/*!< if new segmented objects should be saved for training */
  std::string newImagesDirPath;/*!< path to save new images at */

  std::string savedDataDirPath; /*!< directory for saving data*/
  std::string imagesDirPath;/*!< directory for training images */
  std::string testImagesDirPath;/*!< directory for testing images */

};

/*!
 * Creates and runs the imagerecogniter node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);
#endif //PROJECT_IMAGE_RECOGNIZER_H
