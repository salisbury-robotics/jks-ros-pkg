#include "svlElevatorOverlayer.h"


svlElevatorOverlayer::~svlElevatorOverlayer(){
  cvReleaseImage(&this->image);
}

svlElevatorOverlayer:: svlElevatorOverlayer(string& file, svlObject2dFrame& raw_detections, bool isCallPanel = false): 
  image_file(file),
  image_name((file.find_last_of("/") == string::npos) ? file.substr(0,file.length()-4) : file.substr(file.find_last_of("/")+1,abs(int(file.find_last_of("/")+1-(file.length()-4))))),
  processed_detections(svlObject2dSequence()),
  isCallPanel(isCallPanel),
  image(cvLoadImage(file.c_str()))
{
  this->processed_detections["0_Raw_Detections"] = raw_detections;
}

bool isButtonDetection(svlObject2d& detect, svlObject2d& gt){
  double d_center_x = detect.x + detect.w*0.5;
  double d_center_y = detect.y + detect.h*0.5;
  double gt_center_x = gt.x + gt.w*0.5;
  double gt_center_y = gt.y + gt.h*0.5;
  double x_distance =abs(d_center_x-gt_center_x);
  double y_distance =abs(d_center_y-gt_center_y);
  double x_threshold = gt.w*.25;
  double y_threshold = gt.h*.25;
  
  return ((x_distance > x_threshold)||(y_distance > y_threshold)) ? false : true;
}

void svlElevatorOverlayer::processDetections(){
  svlObject2dFrame d = this->processed_detections["0_Raw_Detections"];
  svlObject2dFrame d2;

  this->processed_detections["1_Pruned"] =this->getPrunedDetections(d);
  d = this->processed_detections["1_Pruned"];
  this->processed_detections["2_Valid"] =this->getValidDetections(d);
  d = this->processed_detections["2_Valid"];
  this->processed_detections["3_Threshold"] =this->getThresholdDetections(d);
  d = this->processed_detections["3_Threshold"];
  this->processed_detections["4_High_Overlap"] =this->getHighOverlapDetections(d);
  d = this->processed_detections["4_High_Overlap"];
  this->final_detections =   this->processed_detections["4_High_Overlap"]; 
  if(this->isCallPanel){
    CallButtonEnhancer cbe;
    this->processed_detections["5_Call_Enhanced"] = cbe.process(this->image,d,this->image->width*(this->image->height)*MAX_RATIO);
    this->final_detections =   this->processed_detections["5_Call_Enhanced"];
  }

}

svlObject2dFrame svlElevatorOverlayer::getFinalDetections(){
  return this->final_detections;
}

bool saveFalsePositives(svlObject2dFrame& detections,svlObject2dFrame& gt, string& image_name,string& neg_dir)
{
  IplImage* fp;
  IplImage* image = cvLoadImage((TEST_IMAGES_PATH + image_name+".jpg").c_str());
  string save_dir = neg_dir;
  bool isFalsePositive;
  svlObject2d detection;
  svlObject2d gt_det;
  vector <int> debug;
  char buffer[33];
  string name ="";
  string outfile;
  double det_x_center;
  double det_y_center;
  double gt_x_center;
  double gt_y_center;
  double gt_area;
  double center_distance;
  for(int d = 0; d< detections.size();d++){
    isFalsePositive = true;
    detection = detections[d];
    for (int g =0; g< gt.size();g++){
      if(isButtonDetection(detection,gt[g])){
	isFalsePositive = false;
	break;
      }
    }
    if(isFalsePositive){
      cvSetImageROI(image, cvRect(detection.x, detection.y, detection.w, detection.h) );
      fp = cvCreateImage(cvSize(detection.w, detection.h), image->depth, image->nChannels);
      cvCopy(image, fp);
      cvResetImageROI(image);  
      name = "";
      snprintf(buffer,32,"%f",detection.x);
      name = name + "x=\"" + string(buffer)+"\" ";
      snprintf(buffer,32,"%f",detection.y);
      name = name + "y=\"" + string(buffer)+"\" ";
      snprintf(buffer,32,"%f",detection.w);
      name = name + "w=\"" + string(buffer)+"\" ";
      snprintf(buffer,32,"%d",d);
      name = name + string(buffer);
      outfile = save_dir + "false_positive_" + image_name +"_" + name + ".jpg";
      if(!cvSaveImage(outfile.c_str() ,fp)){
	printf("Could not save: %s\n",outfile);
	cvReleaseImage(&image);
	return false;
      }
      cvReleaseImage(&fp);
    }
  }
  cvReleaseImage(&image);
  return true;
}


bool createNegatives(svlObject2dFrame& gt, string& image_name,string& neg_dir,int numNegatives)
{
  IplImage* fp;
  IplImage* image = cvLoadImage((TEST_IMAGES_PATH + image_name+".jpg").c_str());
  string save_dir = neg_dir;
  bool isNegative;
  svlObject2d detection = svlObject2d();
  svlObject2d gt_det;
  vector <int> debug;
  char buffer[33];
  string name ="";
  string outfile;
  double det_x_center;
  double det_y_center;
  double gt_x_center;
  double gt_y_center;
  double gt_area;
  double center_distance;
  int createdNegatives = 0;
  while(createdNegatives < numNegatives){
    detection.y = rand() % (5*image->height/6);
    detection.h = rand() % (50) + 130;
    detection.x = rand() % (5*image->width/6);
    detection.w = detection.h*2;
    if(detection.y + detection.h > image->height|| detection.x + detection.w> image->width){
      continue;
    }
    isNegative = true;
    for (int g =0; g< gt.size();g++){
      if(isButtonDetection(detection,gt_det)){
	isNegative = false;
	break;
      }
    }
    if(isNegative){

      name = "";
      snprintf(buffer,32,"%f",detection.x);
      name = name + "x=\"" + string(buffer)+"\" ";
      snprintf(buffer,32,"%f",detection.y);
      name = name + "y=\"" + string(buffer)+"\" ";
      snprintf(buffer,32,"%f",detection.w);
      name = name + "w=\"" + string(buffer)+"\" ";
      snprintf(buffer,32,"%f",detection.h);
      name = name + "h=\"" + string(buffer)+"\" ";
      snprintf(buffer,32,"%d",createdNegatives);
      name = name + string(buffer);
      outfile = save_dir + "negative_" + image_name +"_" + name + ".jpg";
      printf("outfile: %s\n",outfile.c_str());
      cvSetImageROI(image, cvRect(detection.x, detection.y, detection.w, detection.h) );
      fp = cvCreateImage(cvSize(detection.w, detection.h), image->depth, image->nChannels);
      cvCopy(image, fp);
      cvResetImageROI(image);  
      if(!cvSaveImage(outfile.c_str() ,fp)){
	printf("Could not save: %s\n",outfile.c_str());
	cvReleaseImage(&image);
	return false;
      }
      cvReleaseImage(&fp);
      createdNegatives++;
    }
  }
  cvReleaseImage(&image);
  return true;
}

bool svlElevatorOverlayer::saveOverlays(string save_dir)
{
  svlObject2dFrame detections; 
  int size =    this->image->width > 1000 ? 1: 3;
  CvFont font;
  double hScale=1.0;
  double vScale=1.0;
  int    lineWidth=size;
  char buffer[32];
  string pr;
  for(map<string,svlObject2dFrame>::iterator it=this->processed_detections.begin() ; it != this->processed_detections.end(); it++){
    IplImage* img = cvCloneImage(this->image);
    
    detections = (*it).second;
    
    for(int i =0; i<detections.size();i++){
      svlObject2d detection = detections[i];

      snprintf(buffer,3,"%f",detection.pr);
      pr = string(buffer);

      cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
      cvPutText (img,pr.c_str(),cvPoint(detection.x,detection.y), &font, cvScalar(255,255,0));

      // draw a box with red lines of width 1 around detection
      cvRectangle(img, cvPoint(detection.x,detection.y), cvPoint(detection.x+detection.w,detection.y+detection.h), cvScalar(0x0,0x0,0xff), size);
      cvLine(img, cvPoint(detection.x+0.5*detection.w-2,detection.y+0.5*detection.h-2), cvPoint(detection.x+0.5*detection.w+2,detection.y+0.5*detection.h+2), cvScalar(0xff,0x0,0x0), 1);
      cvLine(img, cvPoint(detection.x+0.5*detection.w+2,detection.y+0.5*detection.h-2), cvPoint(detection.x+0.5*detection.w-2,detection.y+0.5*detection.h+2), cvScalar(0xff,0x0,0x0), 1);
    }
    string outfile = (save_dir + (*it).first + ".jpg");


    if(!cvSaveImage(outfile.c_str() ,img)!=0){
      cout << "Could not save: " << outfile << endl;
      cvReleaseImage(&img);
      return false;
    }
    cvReleaseImage(&img);
  }
  return true;
}

svlObject2dFrame svlElevatorOverlayer::getThresholdDetections(svlObject2dFrame& detections)
{
  // calculate threshold
  double threshold = getPrMax(detections) - 1.9*getPrSTD(detections);
  for(svlObject2dFrame::iterator it = detections.begin(); it !=detections.end();){
    if((*it).pr < threshold){
      it = detections.erase(it);
    }else{
      it++;
    }
  }
  return detections;
}

svlObject2dFrame svlElevatorOverlayer::getValidDetections(svlObject2dFrame& detections)
{
  IplImage* gray=cvCreateImage(cvSize(this->image->width,this->image->height),IPL_DEPTH_8U,1);
  cvCvtColor(this->image,gray,CV_BGR2GRAY);;
  bwareaopen(gray,400);
  IplImage* binary= cvCloneImage(gray);
  cvThreshold(gray,binary,1,1,CV_THRESH_BINARY);
  cvReleaseImage(&gray);
  


  CvScalar ul;
  CvScalar ur;
  CvScalar bl;
  CvScalar br;

  
  int u;
  int b;
  int l;
  int r;
  for(svlObject2dFrame::iterator it = detections.begin(); it !=detections.end();){
    b=(*it).y;
    u=(*it).y + (*it).h;
    l=(*it).x;
    r=(*it).x + (*it).w;
    ul=cvGet2D(binary,u,l); // get the uperrleft pixel value
    ur=cvGet2D(binary,u,r); // get the upperright pixel value
    bl=cvGet2D(binary,b,l); // get the bottomleft pixel value
    br=cvGet2D(binary,b,r); // get the bottomright pixel value
    if(ul.val[0] == 0 || ur.val[0] == 0 || bl.val[0] == 0 || br.val[0] == 0){
      it = detections.erase(it);
    }else{
      it++;
    }
  }
  cvReleaseImage(&binary);
  return detections;
}

svlObject2dFrame svlElevatorOverlayer::getPrunedDetections(svlObject2dFrame& detections)
{

  // calculate threshold
  double min_threshold = this->image->width*(this->image->height)*MIN_RATIO;
  double max_threshold = this->image->width*(this->image->height)*MAX_RATIO;
  for(svlObject2dFrame::iterator it = detections.begin(); it !=detections.end();){
    if((*it).w*(*it).h < min_threshold || (*it).w*(*it).h > max_threshold){
      it = detections.erase(it);
    }else{
      it++;
    }
  }
  return detections;
}


svlObject2dFrame svlElevatorOverlayer::getHighOverlapDetections(svlObject2dFrame& detections)
{
  int height = this->image->height;
  int width =  this->image->width;
  
  double* overlay_pr_map = new double[height*width];
  double overlay_pr_max=0;; 
  double overlay_pr_mean=0; 
  double overlay_pr_std; 

  for(vector<svlObject2d>::iterator it= detections.begin(); it !=detections.end();it++){
    for(int x = (*it).x; x < (*it).x + (*it).w;x++){
      for(int y = (*it).y; y < (*it).y + (*it).h;y++){
      	overlay_pr_map[width*y+x]+=(*it).pr;
      }
    }
  }
  for(int y =0; y<height; y++){
    for(int x=0; x<width;x++){
      overlay_pr_mean+= overlay_pr_map[width*y+x];
      if(overlay_pr_map[width*y+x] > overlay_pr_max){
	overlay_pr_max = overlay_pr_map[width*y+x];
      }
    }
  }

  overlay_pr_mean = overlay_pr_mean/(this->image->height*this->image->width);
  for(int y =0; y<height; y++){
    for(int x=0; x<width;x++){
      overlay_pr_std += pow(overlay_pr_map[width*y+x] - overlay_pr_mean,2);
    }
  }

  overlay_pr_std=sqrt(overlay_pr_std/(this->image->height*this->image->width));
  for(svlObject2dFrame::iterator it = detections.begin(); it !=detections.end();){
    if(overlay_pr_map[width*(int((*it).y + 0.5*(*it).h)) + (int((*it).x + 0.5*(*it).w))] <1.5*overlay_pr_std){
      it = detections.erase(it);
    }else{
      it++;
    }
  }
  delete[] overlay_pr_map;
  return detections;
}

double getSizeMin(svlObject2dFrame& detections)
{
  double min =9999999;
  for(int i=0; i<detections.size(); i++){
    if(detections[i].w < min){
      min = detections[i].w*detections[i].h;
    }
  }
  return min;
}


double getPrMax(svlObject2dFrame& detections)
{
  double max =0.0;
  for(int i=0; i<detections.size(); i++){
    if(detections[i].pr > max){
      max = detections[i].pr;
    }
  }
  return max;
}

double getSizeMean(svlObject2dFrame& detections)
{
  double sum=0;
  for(int i=0; i<detections.size(); ++i){
    sum+=detections[i].w*detections[i].h;
  }
  if(detections.size()>0){
    return sum/detections.size();
  }else{
    return 0;
  }
}
double getSizeSTD(svlObject2dFrame& detections)
{
  double mean = getSizeMean(detections);
  double deviation=0;
  for(int i=0; i<detections.size(); ++i){
    deviation+=pow(detections[i].w*detections[i].h-mean,2);
  }
  if(detections.size()>0){
    return sqrt(deviation/detections.size());
  }else{
    return 0;
  }

}
double getPrMean(svlObject2dFrame& detections)
{
  double sum=0;
  for(int i=0; i<detections.size(); ++i){
    sum+=detections[i].pr;
  }
  if(detections.size()>0){
    return sum/detections.size();
  }else{
    return 0;
  }
}

double getPrSTD(svlObject2dFrame& detections)
{
  double mean = getPrMean(detections);
  double deviation=0;
  for(int i=0; i<detections.size(); ++i){
    deviation+=pow(detections[i].pr-mean,2);
  }
  if(detections.size()>0){
    return sqrt(deviation/detections.size());
  }else{
    return 0;
  }
}


int bwareaopen(IplImage *image, int size)
{

  /* OpenCV equivalent of Matlab's bwareaopen.
     image must be 8 bits, 1 channel, black and white
     (objects) with values 0 and 255 respectively */

  CvMemStorage *storage;
  CvSeq *contour = NULL;
  CvScalar white, black;
  IplImage *input = NULL;  // cvFindContours changes the input
  double area;
  int foundCountours = 0;

  black = CV_RGB( 0, 0, 0 );
  white = CV_RGB( 255, 255, 255 );

  if(image == NULL || size == 0)  
    return(foundCountours);

  int i, j;
  // 32 bit image:
  IplImage *Image32 = image;
  // 8 bit image:
  IplImage *Image8 = cvCreateImage(cvSize(100, 100), IPL_DEPTH_8U, 1);

  for(i = 0; i < Image8->width; i++){
    for(j = 0; j < Image8->height; j++){
      CV_IMAGE_ELEM(Image8, uchar, j, i) =CV_IMAGE_ELEM(Image32, float, j, i);
    }
  }

  input = Image8;;
  
  storage = cvCreateMemStorage(0); // pl.Ensure you will have enoughroom here.

  cvFindContours(input, storage, &contour, sizeof (CvContour),
		 CV_RETR_LIST,
		 CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

  while(contour)
    {
      area = cvContourArea(contour, CV_WHOLE_SEQ );  
      if( -size <= area && area <= 0)
	{ // removes white dots
	  cvDrawContours( image, contour, black, black, -1, CV_FILLED, 8 );
	}
      else
	{
	  if( 0 < area && area <= size) // fills in black holes
	    cvDrawContours( image, contour, white, white, -1, CV_FILLED, 8 );
	}
      contour = contour->h_next;
    }

  cvReleaseMemStorage( &storage );  // desallocate CvSeq as well.
  cvReleaseImage(&input);

  return(foundCountours);

}








