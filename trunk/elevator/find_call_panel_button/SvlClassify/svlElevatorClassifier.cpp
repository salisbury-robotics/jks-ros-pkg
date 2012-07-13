#include "svlElevatorClassifier.h"

svlElevatorClassifier::svlElevatorClassifier(string& file, string model, bool isCallPanel, bool save_overlays,string gt_file):
  classifications(svlObject2dFrame()),
  isCallPanel(isCallPanel),
  save_overlays(save_overlays),
  test_image_file(file),
  // if there is no '/' in the file, then the test_set is just file w/o extension, 
  // if there is '/' in file, then return the string between the last one and the extension
  test_image_name((file.find_last_of("/") == string::npos) ? file.substr(0,file.length()-4) : file.substr(file.find_last_of("/")+1,abs(int(file.find_last_of("/")+1-(file.length()-4))))), 
  // if there is no '/' then the file is in current directory
  // if there is '/' in file, then return the base of the file up to just past the last '/'
  gt_file(gt_file),
  model(model)
{
  this->overlays_path = DETECTION_OVERLAYS_PATH +this->test_image_name + "_"+ this->model+"/";
  if(this->save_overlays){
    system(("rm -Rf "+ this->overlays_path).c_str());
    system(("mkdir "+ this->overlays_path).c_str());
  }
};


void svlElevatorClassifier::classify()
{
  svlObject2dSequence image_detections;
  IplImage* image = cvLoadImage(this->test_image_file.c_str());
  char buffer[33];
  snprintf(buffer,32,"%d",image->height);
  string height = string(buffer);
  snprintf(buffer,32,"%d",image->width);
  string width = string(buffer);
  snprintf(buffer,32,"%f",double(image->width)/640.0);
  string scale = "1";//string(buffer);
  string command = SCRIPTS_PATH + "classify.sh " + RAW_DETECTIONS_PATH + " "+ MODELS_PATH + " " + this->model + +" "+ this->test_image_name + "_" +this->model+" " + this->test_image_file + " " + height + " " + width+ " " + scale;
  cout << command << endl;
  cvReleaseImage(&image);
  system(command.c_str());
  string detection_file = RAW_DETECTIONS_PATH + this->test_image_name +"_" +this->model + ".xml";
  bool loaded = readObject2dSequence(detection_file.c_str(), image_detections);
  svlObject2dSequence::iterator found = image_detections.find(this->test_image_name);
  if(loaded && found!= image_detections.end()){
    this->classifications = processDetections((*found).second);
  }else{
    cout << "classify FAILED to load file: \"" << detection_file <<"\"" << endl;
  }
}

svlObject2dFrame svlElevatorClassifier::processDetections(svlObject2dFrame& detections){
  string neg_dir;
  string command;
  svlElevatorOverlayer over = svlElevatorOverlayer(this->test_image_file, detections,this->isCallPanel);
  over.processDetections();
  svlObject2dSequence ground_truths;
  bool loaded = readObject2dSequence(this->gt_file.c_str(), ground_truths);
  string file_base = this->test_image_name +"_"+this->model;
  string file_name;
  bool write;

  file_name = file_base;
  write = writeCacheObject2dFrame((FINAL_DETECTIONS_PATH).c_str(), file_name.c_str(), over.getFinalDetections());
  if(!write){
    cout << "FAILED TO WRITE FINAL DETECTIONS XML: \"" +FINAL_DETECTIONS_PATH +file_name + ".txt\"" << endl;
  }else{
    cout << "Saved FINAL DETCTIONS: \"" << FINAL_DETECTIONS_PATH +file_name + ".txt\"" << endl;
  }
  if(this->save_overlays && !over.saveOverlays(this->overlays_path)){
    cout << "Saving Overlays From: \"" << this->test_image_name + "_" + this->model << ".jpg\" failed" << endl;; 
  }else{
    cout << "Saved Overlays From: \"" << this->test_image_name + "_" + this->model << ".jpg\"" << endl;; 
  }

  return over.getFinalDetections();
}


bool svlElevatorClassifier::createFalsePositives(){
  string gt = this->gt_file;
  if(gt != ""){
    svlObject2dSequence ground_truths;
    bool loaded = readObject2dSequence(gt.c_str(), ground_truths);
    string save_dir = FALSE_POSITIVES_PATH + this->test_image_name +"_"+this->model+ "/";
    string overlay_folder = this->overlays_path;
    string outfile=overlay_folder +"ground_truth.jpg";
    IplImage* img;
    svlObject2d detection;
    svlObject2dFrame gt_detections;
    system(("rm -Rf "+save_dir).c_str());
    system(("mkdir " + save_dir).c_str());
    svlObject2dSequence::iterator found = ground_truths.find(this->test_image_name);
    if(loaded && found != ground_truths.end()){
      gt_detections = (*found).second;
      system(("mkdir " + overlay_folder + this->test_image_name +"/").c_str());
      img = cvLoadImage(this->test_image_file.c_str());
      saveFalsePositives(this->classifications,gt_detections,this->test_image_name,save_dir);
      for (int t =0; t< gt_detections.size();t++){
	detection = gt_detections[t];
	cvRectangle(img, cvPoint(detection.x,detection.y), cvPoint(detection.x+detection.w,detection.y+detection.h), cvScalar(0x0,0xff,0x0), 3);
      }
      if(!cvSaveImage(outfile.c_str() ,img)){
	printf("Could not save: %s\n",outfile);
	return false;
      }
      cvReleaseImage(&img);
    }else{
      cout << "ERROR: COULD NOT LOAD GROUND TRUTH DATA FILE : \"" << gt <<"\""<<endl;
    }
  }
  return true;
}
