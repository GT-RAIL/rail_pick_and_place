//
// Created by bhetherman on 3/30/15.
//

#include <rail_recognition/ImageRecognizer.h>

using namespace cv;
using namespace std;

/*!
 * \brief compares two pair<float, string> objects returning if the first is greater than the second 
 */
bool pairCompare(const std::pair<float, string>& firstElem, const std::pair<float, string>& secondElem) {
  return firstElem.first > secondElem.first;
}

/*!
 * \class ImageRecognizer
 * \brief provides a framework to train and test an ANN for image recognition
 *
 * ImageRecognizer creates a ROS node that provides methods for loading, training and testing
 * a neural network using OpenCV's built in CvANN_MLP class and multiple features
 * that are also implemented in OpenCV
 */
ImageRecognizer::ImageRecognizer() : pnh("~")
{
  //ros::NodeHandle private_nh("~");
  numResponses = 1;
  dictionarySize=10;//the number of bags
  histSize = 10;

  //get parameters and directories of all stored data
  stringstream ssSavedDataDirPath;
  ssSavedDataDirPath << ros::package::getPath("rail_recognition") << "/data/saveddata";
  stringstream ssImagesDirPath;
  ssImagesDirPath << ros::package::getPath("rail_recognition") << "/data/images";
  stringstream ssTestImagesDirPath;
  ssTestImagesDirPath << ros::package::getPath("rail_recognition") << "/data/testimages";
  stringstream ssNewImagesDirPath;
  ssNewImagesDirPath << ros::package::getPath("rail_recognition") << "/data/newimages/";
  pnh.param("saved_data_dir", savedDataDirPath, ssSavedDataDirPath.str());
  pnh.param("images_dir", imagesDirPath, ssImagesDirPath.str());
  pnh.param("test_images_dir", testImagesDirPath, ssTestImagesDirPath.str());
  pnh.param("new_images_dir", newImagesDirPath, ssNewImagesDirPath.str());
  pnh.param("save_new_images", saveNewImages, false);

  bfMatcher = new BFMatcher(NORM_L2, false );//(new FlannBasedMatcher());
  flannMatcher = new FlannBasedMatcher();//new BFMatcher(NORM_HAMMING));

  classifier = new CvANN_MLP();

  if (saveNewImages)
  {
    string segmentedObjectsDefault = "/rail_segmentation/segmented_objects";
    string segmentedObjectsTopic;
    pnh.param("segmented_objects_topic", segmentedObjectsTopic, segmentedObjectsDefault);
    sub = node.subscribe<rail_manipulation_msgs::SegmentedObjectList>(segmentedObjectsTopic, 1,
                                                                      &ImageRecognizer::objectListCallBack, this);
  }
}

/*!
 * \brief Loads a saved set of BOWs and ANN for image recognition
 */
void ImageRecognizer::loadImageRecognizer()
{
  Mat input, descriptor, featuresUnclustered;
  vector<string> FileNames, classes_names;

  FileNode vocabFileNode;
  FileStorage fs;

  classLegend.clear();
  fs.open(savedDataDirPath + "/recognitionNeuralNet.yml", FileStorage::READ);
  FileNode classLegendFileNode = fs["classLegend"];
  read(classLegendFileNode, classLegend);
  fs.release();
  int numClasses = classLegend.size();
  string fp = savedDataDirPath + "/recognitionNeuralNet.yml";
  classifier->load(fp.c_str(), "ANN");

}

void ImageRecognizer::calculateAndSaveFeatures()
{
  Mat featuresUnclustered, response_hist;
  vector<string> FileNames, classes_names;

  getFilesAndClasses(classes_names, FileNames);

  int numClasses = 0;
  map<string,Mat> classes_training_data; //training data for classifiers
  classes_training_data.clear();

  cout << "Order Training Data and Calculate Features" << endl;
  int total_samples = 0;
  for(vector<string>::iterator f = FileNames.begin(), c = classes_names.begin(); f != FileNames.end(); ++f, ++c)
  {

    Mat input = imread(*f, CV_LOAD_IMAGE_COLOR);
    detectExtractComputeFeatures(input, response_hist);

    if(classes_training_data.count(*c) == 0) //not yet created...
    {
      classes_training_data[*c].create(0,response_hist.cols,response_hist.type());
      numClasses++;
    }
    classes_training_data[*c].push_back(response_hist);
    total_samples++;
  }

  printf("total_samples = %d\n", total_samples);

  //remove duplicates from classes
  sort(classes_names.begin(), classes_names.end());
  classes_names.erase(std::unique(classes_names.begin(), classes_names.end()), classes_names.end());
  ofstream outputFile;
  outputFile.open("feature_vectors.txt", ios::out | ios::app);

  for (unsigned int i = 0; i < classes_names.size(); i ++)
  {
    Mat feature_vectors = classes_training_data[classes_names[i]];
    for (unsigned int j = 0; j < feature_vectors.rows; j ++)
    {
      outputFile << classes_names[i] << ",";

      for (unsigned int k = 0; k < feature_vectors.cols; k ++)
      {
        outputFile << feature_vectors.at<float>(j, k);
        if (k < feature_vectors.cols - 1)
          outputFile << ",";
      }
      outputFile << "\n";
    }
  }
  outputFile.close();
  printf("Feature vectors saved.");
}

/*!
 * \brief Trains the BOWs and the ANN using the images in the images directory
 */
void ImageRecognizer::trainImageRecognizer()
{
  Mat featuresUnclustered, response_hist;
  vector<string> FileNames, classes_names;

  getFilesAndClasses(classes_names, FileNames);

  int numClasses = 0;
  map<string,Mat> classes_training_data; //training data for classifiers
  classes_training_data.clear();

  cout << "Order Training Data and Calculate Features" << endl;
  int total_samples = 0;
  for(vector<string>::iterator f = FileNames.begin(), c = classes_names.begin(); f != FileNames.end(); ++f, ++c)
  {
    Mat input = imread(*f, CV_LOAD_IMAGE_COLOR);
    detectExtractComputeFeatures(input, response_hist);

    if(classes_training_data.count(*c) == 0) //not yet created...
    {
      classes_training_data[*c].create(0,response_hist.cols,response_hist.type());
      numClasses++;
    }
    classes_training_data[*c].push_back(response_hist);
    total_samples++;
  }

  printf("total_samples = %d\n", total_samples);

  printf("Training Neural Net\n");
  classLegend.clear();
  for (map<string,Mat>::iterator it = classes_training_data.begin(); it != classes_training_data.end(); ++it) {
    classLegend.push_back((*it).first);
  }

  classifier = new CvANN_MLP();
  trainNN(response_hist, classes_training_data,numClasses);
  FileStorage fs(savedDataDirPath + "/recognitionNeuralNet.yml", FileStorage::APPEND);//store the vocabulary
  fs << "classLegend" << classLegend;
  fs.release();
  printf("Training Done\n");
}

/*!
 * \brief Tests image recognition with the images in the test directory
 */
void ImageRecognizer::testImageRecognizer()
{
  printf("Testing Start\n");
//evaluate
  Mat test_response_hist, res;

  cout << "classLegend : [";
  for(vector<string>::iterator it = classLegend.begin(); it != classLegend.end(); ++it)
    cout << *it << ", ";
  cout << "]" << endl;
  DIR *testDp = opendir(testImagesDirPath.c_str());

  struct dirent *testEntry = readdir(testDp);

  int right, totalRight=0, count, totalCount=0;


  map<string,int> confusionMap; //training data for classifiers
  confusionMap.clear();
  for (int i = 0; i < classLegend.size(); i++)
  {
    confusionMap[classLegend.at(i)] = i;
  }
  int actual = 0, predicted = 0;
  Mat confusionMatrix = Mat::zeros(classLegend.size(), classLegend.size(), CV_32F);

  while (testEntry != NULL)
  {
    if (testEntry->d_type == DT_DIR && strcmp(testEntry->d_name, ".") != 0 && strcmp(testEntry->d_name, "..") != 0)
    {
      actual = confusionMap[testEntry->d_name];
      string classDirPath = testImagesDirPath + "/" + testEntry->d_name;
      printf("classDirPath: %s\n", classDirPath.c_str());

      DIR *classTestDir = opendir(classDirPath.c_str());
      struct dirent *classEntry = readdir(classTestDir);
      right = 0;
      count = 0;
      while (classEntry != NULL)
      {
        if (classEntry->d_type == DT_DIR || strcmp(classEntry->d_name, ".") == 0 || strcmp(classEntry->d_name, "..") == 0)//|| strstr(classEntry->d_name, ".png") == NULL)
        {
          classEntry = readdir(classTestDir);
          continue;
        }
        string testfilepath = classDirPath + "/" + classEntry->d_name;

        Mat testInput = imread(testfilepath, CV_LOAD_IMAGE_COLOR);
        detectExtractComputeFeatures(testInput, test_response_hist);
        classifier->predict(test_response_hist, res);

        vector<pair<float, string> > vRes;
        for (int i = 0; i < res.cols; i++)
        {
          vRes.push_back(std::make_pair(res.at<float>(i),classLegend.at(i)));
        }
        sort(vRes.begin(), vRes.end(), pairCompare);

        int bestMatchIndex = -1;

        count++;
        for (int i = 0; i < numResponses; i++)
        {
          if(strcmp(vRes.at(i).second.c_str(), testEntry->d_name) == 0)
          {
            bestMatchIndex = i;
          }
        }

        if(bestMatchIndex == -1)
        {
          predicted = confusionMap[vRes.at(0).second];
          confusionMatrix.at<float>(actual,predicted) += 1;

          cout << classEntry->d_name << " : WRONG : " << endl;
          cout << "classesMap : [";
          for(vector<pair<float, string> >::iterator it = vRes.begin(); it != vRes.end(); ++it)
            cout << (*it).second << ", ";
          cout << "]" << endl;
          cout << "vRes : [";
          for(vector<pair<float, string> >::iterator it = vRes.begin(); it != vRes.end(); ++it)
            cout << (*it).first << ", ";
          cout << "]" << endl;
        }
        else
        {
          confusionMatrix.at<float>(actual,actual) += 1;
          cout << classEntry->d_name << " : CORRECT : " << vRes.at(0).first << endl;//<< res << endl;
          right++;
        }
        classEntry = readdir(classTestDir);
      }


      cout << testEntry->d_name << " : CORRECT = " << right << " : WRONG = " << count-right << " : TOTAL CLASS = " << count << endl << endl;
      totalRight += right;
      totalCount += count;

      closedir(classTestDir);
    }
    testEntry = readdir(testDp);
  }
  cout << endl;

  cout << "confusionMatrix = "<< endl << " "  << confusionMatrix << endl << endl;
  cout << "TOTAL CORRECT = " << totalRight << " : TOTAL WRONG = " << totalCount-totalRight;
  cout << " : TOTAL ALL = " << totalCount << " --- " << (float)totalRight/totalCount*100 << " %" << endl << endl;
  closedir(testDp);
}

/*!
 * \brief Detects and extracts keypoints and a descriptor
 *
 * \param input The gray image to use for destection
 * \param keypoints A pointer to a keypoint vector to store the found keypoints.
 * \param extractor A pointer to the extractor to use.
 * \param detector A pointer to the detector to use.
 * \return None
 */
void ImageRecognizer::detectExtractFeatures(Mat &input, vector<KeyPoint> &keypoints, Mat &descriptor, Ptr<DescriptorExtractor> extractor, Ptr<FeatureDetector> detector)
{
  detector->detect(input, keypoints);//detect feature points
  extractor->compute(input, keypoints, descriptor);//compute the descriptors for each keypoint
}

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
void ImageRecognizer::detectExtractComputeFeatures(Mat colorInput, Mat &response_hist)
{
  vector<KeyPoint> keypoints;
  Mat colorhsv_hist, inputGS, descriptor;

  cvtColor(colorInput,inputGS,CV_RGB2GRAY);

  colorAndHSVDescriptors(colorInput, colorhsv_hist);

  response_hist = colorhsv_hist.clone();
}

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
void ImageRecognizer::getFeatures(Mat &featuresUnclustered, Ptr<DescriptorExtractor> extractor,
                                  Ptr<FeatureDetector> detector, vector<string> &FileNames)
{
  Mat input;//to store the current input image
  vector<KeyPoint> keypoints;//To store the keypoints that will be extracted by SIFT
  Mat descriptor;//To store the SIFT descriptor of current image

  featuresUnclustered = Mat();
  for(vector<string>::iterator f = FileNames.begin(); f != FileNames.end(); ++f)
  {
    Mat input = imread(*f, CV_LOAD_IMAGE_GRAYSCALE); //Load as grayscale
    detectExtractFeatures(input, keypoints, descriptor, extractor, detector);
    featuresUnclustered.push_back(descriptor);
  }
}

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
void ImageRecognizer::getFilesAndClasses(vector<string> &classes_names, vector<string> &FileNames)
{
  FileNames.clear();
  classes_names.clear();
  DIR *dir = opendir(imagesDirPath.c_str());

  struct dirent *entry = readdir(dir);

  while (entry != NULL)
  {
    if (entry->d_type == DT_DIR && strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
    {
      string classDirPath = imagesDirPath + "/" + entry->d_name;
      DIR *classDir = opendir(classDirPath.c_str());
      struct dirent *classEntry = readdir(classDir);

      while (classEntry != NULL)
      {
        if (classEntry->d_type == DT_DIR || strcmp(classEntry->d_name, ".") == 0 || strcmp(classEntry->d_name, "..") == 0)
        {
          classEntry = readdir(classDir);
          continue;
        }

        string filepath = classDirPath + "/" + classEntry->d_name;
        FileNames.push_back(filepath);
        classes_names.push_back(entry->d_name);
        classEntry = readdir(classDir);
      }
      closedir(classDir);
    }
    entry = readdir(dir);
  }
  closedir(dir);
}

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
void ImageRecognizer::trainVocabulary(string featureName, Mat featuresUnclustered, Mat &vocabulary)
{

  printf("BOWKMeansTrainer\n");
  TermCriteria tc(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,1000000,0.000001);//define Term Criteria
  int retries=1;//retries number
  int flags=KMEANS_PP_CENTERS;//necessary flags

  Mat featuresUnclustered_32f;
  featuresUnclustered.convertTo(featuresUnclustered_32f, CV_32F);
  BOWKMeansTrainer bowTrainer(dictionarySize,tc,retries,flags);//Create the BoW (or BoF) trainer
  bowTrainer.add(featuresUnclustered_32f);
  vocabulary=bowTrainer.cluster();//cluster the feature vectors

  FileStorage fs(savedDataDirPath + "/" + featureName + "_dictionary.yml", FileStorage::WRITE);//store the vocabulary
  fs << "vocabulary" << vocabulary;
  fs.release();
}

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
void ImageRecognizer::trainNN(Mat &inputData, map<string,Mat> &classesTrainingData, int numClasses)
{
  Mat layers = Mat(3, 1, CV_32S);
  layers.row(0)  = inputData.cols;
  layers.row(1)  = inputData.cols*2;
  layers.row(2)  = numClasses;
  cout << "inputData.cols : " << inputData.cols << endl;

  CvANN_MLP_TrainParams params;
  TermCriteria criteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,500000,0.0000001);

  params.train_method = CvANN_MLP_TrainParams::BACKPROP;
  params.bp_dw_scale  = 0.1;
  params.bp_moment_scale = 0.05;
  params.term_crit  = criteria;

  classifier->create(layers, CvANN_MLP::SIGMOID_SYM,1,1);

  int index = 0;
  Mat samples(0,inputData.cols,inputData.type());
  Mat labels(0,numClasses,CV_32F);

  for (map<string,Mat>::iterator it = classesTrainingData.begin(); it != classesTrainingData.end(); ++it) {
    string class_ = (*it).first;
    samples.push_back(classesTrainingData[class_]);
    Mat class_label = Mat::ones(classesTrainingData[class_].rows, numClasses, CV_32F);
    class_label *= -1;
    class_label.col(index) += 2.0;
    labels.push_back(class_label);

    index++;
  }

  Mat samples_32f;
  samples.convertTo(samples_32f, CV_32F);

  classifier->train(samples_32f,labels, cv::Mat(), cv::Mat(), params);
  string sp = savedDataDirPath + "/recognitionNeuralNet.yml";
  classifier->save(sp.c_str(), "ANN");
}

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
void ImageRecognizer::colorAndHSVDescriptors(Mat colorInput, Mat &result_hist)
{
  float hsvRange[] = { 0, 180 } ; //the upper boundary is exclusive
  float range[] = { 0, 256 } ; //the upper boundary is exclusive
  const float* hsvHistRange = { hsvRange };
  const float* histRange = { range };
  bool uniform = true; bool accumulate = false;
  Mat b_hist, g_hist, r_hist, hue_hist, sat_hist, val_hist;
  Mat temp_results, hsv;
  vector<Mat> bgr_planes, hsv_planes;

  if(!colorInput.data )
  {
    cout << "No image data" << endl;
    return;
  }

  split( colorInput, bgr_planes );

  cvtColor( colorInput, hsv, CV_RGB2HSV );
  split( hsv, hsv_planes );

  /// Compute the histograms:
  calcHist( &hsv_planes[0], 1, 0, Mat(), hue_hist, 1, &histSize, &hsvHistRange, uniform, accumulate );
  calcHist( &hsv_planes[1], 1, 0, Mat(), sat_hist, 1, &histSize, &hsvHistRange, uniform, accumulate );
  calcHist( &hsv_planes[2], 1, 0, Mat(), val_hist, 1, &histSize, &hsvHistRange, uniform, accumulate );

  calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

  /// Normalize the results
  int numPixels = colorInput.size().height*colorInput.size().width;
  normalizeFeatureVector(hue_hist, numPixels);
  normalizeFeatureVector(sat_hist, numPixels);
  normalizeFeatureVector(val_hist, numPixels);
  normalizeFeatureVector(b_hist, numPixels);
  normalizeFeatureVector(g_hist, numPixels);
  normalizeFeatureVector(r_hist, numPixels);

  temp_results = hue_hist.t();
  hconcat(temp_results, sat_hist.t(),temp_results);
  hconcat(temp_results, val_hist.t(),temp_results);

  hconcat(temp_results,b_hist.t(),temp_results);
  hconcat(temp_results,g_hist.t(),temp_results);
  hconcat(temp_results,r_hist.t(),temp_results);

  result_hist = temp_results.clone();
}

void ImageRecognizer::normalizeFeatureVector(Mat &featureVector, int numPixels)
{
  for (unsigned int i = 0; i < featureVector.rows; i ++) {
    for (unsigned int j = 0; j < featureVector.cols; j++) {
      featureVector.at<float>(i, j) /= ((float)numPixels);
    }
  }
}

/*!
 * \brief Callback for the new segmented objects list.
 *
 * Performs recognition on each of the images in the list and will save them to the specified
 * directory when the save option is set.
 *
 * \param msg The published segmented objects list being received.
 * \return None
 */
void ImageRecognizer::objectListCallBack(const rail_manipulation_msgs::SegmentedObjectList msg)
{
  int n = 0;
  time_t seconds;
  seconds = time (NULL);

  rail_manipulation_msgs::SegmentedObjectList newObjectList;
  newObjectList.header = msg.header;
  newObjectList.objects.resize(msg.objects.size());
  //cout << "newObjectList.objects.size() = " << newObjectList.objects.size() << endl;
  for (unsigned int i = 0; i < newObjectList.objects.size(); i++)
  {
    newObjectList.objects[i] = msg.objects[i];

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      //cv_ptr = cv_bridge::toCvCopy(newObjectList.objects[i].image, sensor_msgs::image_encodings::RGB8);
      cv_ptr = cv_bridge::toCvCopy(newObjectList.objects[i].image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      continue;
    }

    //save image
    n++;
    stringstream savePath;
    savePath << newImagesDirPath << seconds << "_" << n << ".png";
    imwrite(savePath.str(), cv_ptr->image);
  }
}

void ImageRecognizer::recognizeObject(const rail_manipulation_msgs::SegmentedObject object, vector< pair<float, string> > &recognitionResults)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    //cv_ptr = cv_bridge::toCvCopy(object.image, sensor_msgs::image_encodings::RGB8);
    cv_ptr = cv_bridge::toCvCopy(object.image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  testImage(cv_ptr->image,recognitionResults);
}

/*!
 * \brief Attempts to recognize a given image
 *
 * Tests the given image by calculating its feature vector and then using the ANN to predict.
 *
 * \param colorInput The color image that is to be tested
 * \param testResults A pointer to a vector in which to store the prediction results
 * \return None
 */
void ImageRecognizer::testImage(Mat colorInput, vector<pair<float, string> > &testResults)
{
  Mat testResponseHist, res;
  detectExtractComputeFeatures(colorInput, testResponseHist);
  classifier->predict(testResponseHist, res);

  testResults.clear();
  for (int i = 0; i < res.cols; i++)
  {
    testResults.push_back(std::make_pair(res.at<float>(i),classLegend.at(i)));
  }
  sort(testResults.begin(), testResults.end(), pairCompare);
}
