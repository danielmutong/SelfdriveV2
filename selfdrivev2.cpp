#define POSY0 175
#define YCOORD 240
#define MIDPOINT 300
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Intrinsic.h>
#include <X11/extensions/XTest.h>
#include <math.h>

extern "C"{
#include <unistd.h>
#include <xdo.h>
#include "uhidkb.c"
}

using namespace cv;
using namespace std;

#define M_THRES 0.2
#define B_THRES 10
#define D_THRES 5000



class single_line
{
  
public:
  int x0;
  int x1;
  int y0;
  int y1;
  float m;
  float y_int;
  bool mergeflag = false;
  
  void find_slope(Vec4i);
  void output(void);
  bool merge(single_line otherline, float b_thres, float m_thres, float d_thres, float max_dist);
  double distance_calc_min(single_line otherline );
  double distance_calc_max(single_line otherline);  
  void find_slope_second_round(Vec4i l);

};

int control_ana( std::vector<cv::Vec4i> ,float, float , float, float, Mat, single_line [] );


class road
{

public:
  Mat img;
  Mat roi;
  Mat original;
  int i=0;
  int k = 0;

  int merged_lines_size;
  single_line all_merged_lines [500];
  single_line final_y [50][500];
  vector <Vec4i> temp_left_bounds, temp_right_bounds;
  single_line left_bounds [20];
  single_line right_bounds [20];
  int final_size[50] = {0} ;
  int count   =  0;
  float left_bound[20], right_bound[20];
  
  void get_x_coord(void);
  void sort_x(void);
  void get_bounds(void);
  int y_val=250;
  float x_coord[50][200];
  single_line lines[500];
  void detect_road(cv::Mat, single_line []);
  void getslopes(vector <Vec4i>);
  void connect_bounds(void);

};


void road::detect_road(cv::Mat src, single_line merged_lines [])
{
  
  float m_thres = M_THRES;
  float b_thres = B_THRES;
  float d_thres = D_THRES;
  float max_dist=0;
  Mat original2;

  Mat dst, cdstP, cdst;
  Mat src1, src2, mask, fin_src;
  cv::cvtColor(src,src2, cv::COLOR_BGR2HSV);
  cv::cvtColor(src,src1,cv::COLOR_BGR2GRAY);

  img = src;
  original = img.clone();
  original2 = img.clone();
  Rect lane(10,200,580,90);
  Mat srclane = img(lane);
  imshow("srclane", srclane);

  // Edge detection
  Canny(srclane, dst, 100,200, 3);  
  cvtColor(dst, cdst, COLOR_GRAY2BGR); 
  cdstP = cdst.clone();

  // Probabilistic Line Transform
  vector<Vec4i> linesP; 
  HoughLinesP(dst, linesP, 1, CV_PI/180, 20,10, 10 );
  
  for( size_t i = 0; i < linesP.size(); i++ )
    {
      Vec4i l = linesP[i];
      line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA);
      line( original, Point(l[0]+10, l[1]+200), Point(l[2]+10, l[3]+200), Scalar(0,0,255), 1, LINE_AA);
    }
  
  imshow( "original " , original);
  merged_lines_size=control_ana(linesP, m_thres, b_thres, d_thres,max_dist, img, merged_lines);

  for (int i = 0 ; i < merged_lines_size ; i++)
    {
      all_merged_lines[i] = merged_lines[i];
    }
  
  y_val = 220;
  
  for (y_val = 220; y_val <251; y_val += 5)
    {
      get_x_coord();
      count++;
    }

  count = 0;
  
  for (count = 0; count < 5; count++)
    {
      sort_x();
    }

  for (count = 0; count < 5; count++)
    {
      get_bounds();    
    }

  int j = 0;
  
  for (y_val = 220; y_val <251; y_val += 5)
    {

      char c[10];
      char c1[10];
      snprintf(c,10,"%d",(int) left_bound[j]);
      snprintf(c1,10,"%d",(int) right_bound[j]);

      putText(img, c, cv::Point(left_bound[j],y_val),  FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, LINE_AA);
      putText(img, c1, cv::Point(right_bound[j],y_val),  FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200,200,250), 1, LINE_AA);

      j++;
    }

  connect_bounds();

  for( size_t i = 0; i < 4; i++ )
    {
      line( original2, Point(left_bounds[i].x0, left_bounds[i].y0), Point(left_bounds[i].x1, left_bounds[i].y1), Scalar(0,0,255), 1, LINE_AA);
      line( original2, Point(right_bounds[i].x0, right_bounds[i].y0), Point(right_bounds[i].x1, right_bounds[i].y1), Scalar(0,0,255), 1, LINE_AA);
    }

  imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
  getslopes(linesP);

  imshow("original", img);
  imshow("bounds", original2);

}


void road::getslopes(vector<Vec4i> lines)
{

  float m[lines.size()];
  float y_int[lines.size()];

  for (int i = 0; i < lines.size(); i++)
    {
      Vec4i l = lines[i];
      m[i] = (float)(l[3] - l[2]) / (l[1] - l[0]);
      y_int[i]=l[2] - m[i] * l[0];
    }

}


void road::get_x_coord(void)
{
  
  int j = 0;
  
  for (int i = 0; i < merged_lines_size ; i++){
    
    if (((all_merged_lines[i].y0  ) >  y_val ) && ((all_merged_lines[i].y1 )  < y_val ))
      {
	x_coord[count][j] = (y_val - all_merged_lines[i].y_int) / all_merged_lines[i].m;
	final_y[count][j] = all_merged_lines[i];
	j++;
      }
    
  }
  final_size[count] = j;
}


void road::sort_x(void)
{
  
  for(int i = 0; i < final_size[count]; i++)
    {		
      for(int j = i + 1; j < final_size[count]; j++)
	{
	  if(x_coord[count][i] > x_coord[count][j])
	    {
	      float temp = x_coord[count][i];
	      x_coord[count][i] = x_coord[count][j];
	      x_coord[count][j] = temp;
	    }
	}
    }
}


void road::get_bounds(void)
{

  if (final_size[count] == 0)
    {
      right_bound[count] = 600;
      left_bound[count] = 0;
      k = 1;
    }
  else{
    
    for(int i = 0; i < final_size[count]; i++)
      {

	if (x_coord[count][i] > 300)
	  {
	    k = 2;
	    right_bound[count] = x_coord[count][i];
	    left_bound[count] = x_coord[count][i-1];
	    break;
	  }
      
	else if (x_coord[count][0] > 300)
	  {
	    k = 3;
	    right_bound[count] = x_coord[count][0];
	    left_bound[count] = 0;
	  }
      
	else if (x_coord[count][final_size[count]-1] < 300)
	  {
	    k = 4;
	    right_bound[count] = 600;
	    left_bound[count] = x_coord[count][final_size[count]-1];     
	  }
      
	else
	  {
	    k = 5;
	    right_bound[count] = 600;
	    left_bound[count] = 0;
     
	  }
      }
  }  
}


void road::connect_bounds(void)
{
 
  int y_val = 220;
  
  for ( int i = 0; i < 4; i ++)
    {
      Vec4i l;
      Vec4i r;
      l[0] = left_bound[i];
      l[2] = left_bound[i + 1];
      l[1] = y_val;
      l[3] = y_val + 5;
      left_bounds[i].find_slope_second_round(l);

      r[0] = right_bound[i];
      r[2] = right_bound[i + 1];
      r[1] = y_val;
      r[3] = y_val + 5;
      right_bounds[i].find_slope_second_round(r);

      y_val += 5;    
    }
}
  

void single_line::find_slope(Vec4i l)
{

  if(l[1] < l[3])
    {
      int temp_y = l[1];
      l[1] = l[3];
      l[3] = temp_y;
      int temp_x = l[0];
      l[0] = l[2];
      l[2] = temp_x;
    }
    
  x0 = l[0] + 10;
  x1 = l[2] + 10;
  y0 = l[1] + 200;
  y1 = l[3] + 200;
  m = (float)(y1 - y0) / (x1 - x0);
  y_int=y0-m*x0;
  
}


void single_line::find_slope_second_round(Vec4i l)
{

  if(l[1] < l[3])
    {
      int temp_y = l[1];
      l[1] = l[3];
      l[3] = temp_y;
      int temp_x = l[0];
      l[0] = l[2];
      l[2] = temp_x;
    }
    
  x0 = l[0];
  x1 = l[2];
  y0 = l[1];
  y1 = l[3];
  m = (float)(y1 - y0) / (x1 - x0);
  y_int=y0-m*x0;
  
}


void single_line::output(void)
{
  cout<< x0<<" "<<y0<<" "<<x1<<" "<<y1<<" "<<m<<" "<<y_int<<endl;
}


bool single_line::merge(single_line b, float b_thres, float m_thres, float d_thres, float max_dist)
{
  if ((abs(m - b.m) < m_thres)
      && (abs(y_int - b.y_int) < b_thres)
      && (this->distance_calc_min(b) < d_thres))
    {      
      float max = distance_calc_max(b);
      return true;
    }

  return false;
    
}


double single_line:: distance_calc_min(single_line b)
{
  
  double dist[4];
  dist[0] = pow((x0 - b.x0), 2) + pow((y0 - b.y0), 2);
  dist[1] = pow((x0 - b.x1), 2) + pow((y0 - b.y1), 2);
  dist[2] = pow((x1 - b.x0), 2) + pow((y1 - b.y0), 2);
  dist[3] = pow((x1 - b.x1), 2) + pow((y1 - b.y1), 2);

  double min=dist[0];

  for(int i=1; i < 4; i++)
    {
      if (dist[i] < min)
	{
	  min = dist[i];
	}
    }

  return min;
  
}


double single_line:: distance_calc_max(single_line b)
{
  
  double dist[6];
  dist[0] = pow((x0 - b.x0), 2) + pow((y0 - b.y0), 2);
  dist[1] = pow((x0 - b.x1), 2) + pow((y0 - b.y1), 2); 
  dist[2] = pow((x1 - b.x0), 2) + pow((y1 - b.y0), 2);
  dist[3] = pow((x1 - b.x1), 2) + pow((y1 - b.y1), 2);
  dist[4] = pow((x0 - x1), 2) + pow((y0-y1),2);
  dist[5] = pow((b.x0 - b.x1), 2) + pow((b.y0-b.y1),2);

  int j = 0;
  double max = dist[0];

  for(int i = 1; i < 6; i++)
    {
      if (dist[i] > max)
	{
	  max = dist[i];
	  j=i;
	}
    }

  switch(j){
    
  case 0:
    
    if(y0 > b.y0)
      {
	y0 = y0;
	y1 = b.y0;
	x0 = x0;
	x1 = b.x1;
	m = (float)(y1 - y0) / (x1 - x0);
	y_int=y0-m*x0;
      }
    
    else
      {
	y1 = y0;
	y0 = b.y0;
	x1 = x0;
	x0 = b.x0;
     
	m = (float)(y1 - y0) / (x1 - x0);
	y_int = y0 - m * x0;

      }
    break;
    
  case 1:
    
    if(y0 > b.y1)
      {
	y0 = y0;
	y1 = b.y1;
	x0 = x0;
	x1 = b.x1;
	m = (float)(y1 - y0) / (x1 - x0);
	y_int = y0 - m * x0;
      }
    
    else
      {
	y1=y0;
	y0=b.y1;
	x1=x0;
	x0=b.x1;
    
	m=(float)(y1-y0)/(x1-x0);
	y_int=y0-m*x0;
      }
    break;
    
  case 2:
    
    if(y1>b.y0)
      {
	y0=y1;
	y1=b.y0;
	x0=x1;
	x1=b.x0;
	m=(float)(y1-y0)/(x1-x0);
	y_int=y0-m*x0;
      }
    
    else
      {
	y0 = b.y0;
	y1 = y1;
	x0 = b.x0;
	x1 = x1;
	m = (float)(y1 - y0) / (x1 - x0);
	y_int = y0 - m * x0;
      }
    break;
    
  case 3:
    
    if(y1 > b.y1)
      {
	y0 = y1;
	y1 = b.y1;
	x0 = x1;
	x1 = b.x1;
	m = (float)(y1 - y0) / (x1 - x0);
	y_int = y0 - m * x0;
      }
    
    else
      {
	y0 = b.y1;
	y1 = y1;
	x0 = b.x1;
	x1 = x1;
	m = (float)(y1 - y0) / (x1 - x0);
	y_int = y0 - m * x0;
      }
    break;
    
  case 4:
    
    if (y0 > y1)
      {
	y0 = y0;
	y1 = y1;
	x0 = x0;
	x1 = x1;
	m = (float)(y1 - y0) / (x1 - x0);
	y_int = y0 - m * x0;
      }
    
    else
      {
	int temp = y1;
	int temp2 = x1;
	y1 = y0;
	y0 = temp;
	x1 = x0;
	x0 = temp2;
	m = (float)(y1 - y0) / (x1 - x0);
	y_int = y0 - m * x0;
      }
    break;

  case 5:
    
    if (b.y0 > b.y1)
      {
	y0 = b.y0;
	y1 = b.y1;
	x0 = b.x0;
	x1 = b.x1;
	m = (float)(y1 - y0) / (x1 - x0);
	y_int = y0 - m * x0;
      }
    
    else
      {
	int temp = b.y1;
	int temp2 = b.x1;
	y1 = b.y0;
	y0 = temp;
	x1 = b.x0;
	x0 = temp2;
	m = (float)(y1 - y0) / (x1 - x0);
	y_int = y0 - m * x0;
      }
    break;
  }
  
  return dist[j];

}


void sort_slopes(single_line all_lines[], int original_number_of_lines){
    
  for (int i = 0; i < original_number_of_lines; i++)
    {
      for (int j = i + 1; j < original_number_of_lines; j++)
	{
	  if (all_lines[i].m > all_lines[j].m) 
	    {
	      single_line a=  all_lines[i];
	      all_lines[i] = all_lines[j];
	      all_lines[j] = a;
	    }
	}
    }
  
}

int merge_line(single_line all_lines[], int num_of_lines,float b_thres, float m_thres, float d_thres, float max_dist, single_line new_line[], Mat a){

  int num_of_new_lines;

  for(int i = 0; i < num_of_lines-1; i++)
    {
      if (all_lines[i].mergeflag == false)
	{
	  for (int j = i+1; j < num_of_lines; j++)
	    {
	      if (all_lines[j].mergeflag == false)
		{
		  if (all_lines[i].merge(all_lines[j], b_thres, m_thres, d_thres, max_dist))
		    {
		      all_lines[j].mergeflag = true;
		    }
		  num_of_new_lines++;
		}
	    }
	}
    }
  int new_line_size = 0;
  
  for(int i = 0; i < num_of_lines; i++)
    {
      if (all_lines[i].mergeflag == false)
	{
	  new_line[new_line_size] = all_lines[i];
	  new_line_size++;
	}
    }
  
  return new_line_size;
  
}


int control_ana( std::vector<cv::Vec4i> lines,float m_thres, float b_thres, float d_thres, float max_dist, Mat a, single_line merged_lines [] ){
  
  single_line all_lines[1000];
  int original_number_of_lines = 0;
  for (int i = 0; i < lines.size(); i++)
    {
      Vec4i l = lines[i];
      all_lines[i].find_slope(l);
      original_number_of_lines++;
    }
  
  for(int i = 0; i < original_number_of_lines; i++ )
    {
      sort_slopes(all_lines, original_number_of_lines);
    }
  
  single_line new_line[256];
  
  int new_line_count=0;
  int roundcnt = 0;

  single_line round1merge_arr[500];
  single_line round2merge_arr[500];
  int round1merge_num = merge_line(all_lines, original_number_of_lines, b_thres,m_thres, d_thres, max_dist, round1merge_arr,a);
  int round2merge_num = merge_line(round1merge_arr, round1merge_num,b_thres,m_thres, d_thres, max_dist, round2merge_arr,a);

  vector<Vec4i> line_final;
  
  for(int i = 0; i < round2merge_num; i++ )
    {
      cv::line( a, Point(round2merge_arr[i].x0  , round2merge_arr[i].y0 ), Point(round2merge_arr[i].x1 ,round2merge_arr[i].y1 ), Scalar(0,0,255), 1, LINE_AA);
    }

  imshow("merged",a);
 
  for (int i = 0 ; i < round2merge_num ; i++)
    {
      merged_lines[i] = round2merge_arr[i];
    }
  
  return round2merge_num;

}


struct ScreenShot
{
  ScreenShot(const char* name, int x, int y, int width, int height):
    x(x),
    y(y),
    width(width),
    height(height)
  {

    display = XOpenDisplay(nullptr);
    Window *list;
    xdo_search_t search;
    unsigned int nwindows;
    memset(&search, 0, sizeof(xdo_search_t));
    search.max_depth = -1;
    search.require = xdo_search::SEARCH_ANY;

    search.searchmask |= SEARCH_NAME;
    search.winname = name;

    xdo_t* p_xdo = xdo_new(NULL);
    int id = xdo_search_windows(p_xdo, &search, &list, &nwindows);

    if (list == NULL) {
      cout << " not found " << endl;
    } else
      cout << " total =  " << nwindows << endl;
    root = list[0];
    init = true;
  }

  void operator() (Mat& cvImg)
  {
    if(init == true)
      init = false;
    else
      XDestroyImage(img);

    img = XGetImage(display, root, x, y, width, height, AllPlanes, ZPixmap);

    cvImg = Mat(height, width, CV_8UC4, img->data);
  }

  ~ScreenShot()
  {
    if(init == false)
      XDestroyImage(img);

    XCloseDisplay(display);
  }

  Display* display;
  Window root;
  int x,y,width,height;
  XImage* img;

  bool init;
};

#if 0
static void SendKey (Display * disp, KeySym keysym, KeySym modsym)
{

  KeyCode keycode = 0, modcode = 0;
  keycode = XKeysymToKeycode (disp, keysym);
  if (keycode == 0) return;
  XTestGrabControl (disp, True);
  /* Generate modkey press */

  if (modsym != 0)
    {
      modcode = XKeysymToKeycode(disp, modsym);
      XTestFakeKeyEvent (disp, modcode, True, 0);
    }
  /* Generate regular key press and release */
  XTestFakeKeyEvent (disp, keycode, True, 50);
  XTestFakeKeyEvent (disp, keycode, False, 0);

  /* Generate modkey release */
  if (modsym != 0)
    XTestFakeKeyEvent (disp, modcode, False, 0);

  XSync (disp, False);
  XTestGrabControl (disp, False);
  
}
#endif


class selfdrive
{
  
public:
  float difference[2];
  float right[2],left[2];
  void self_drive_control(single_line [], single_line []);
  void key_press(void);
  
};


void selfdrive::self_drive_control(single_line left_bound[], single_line right_bound[]){

  cout<< abs(abs(left_bound[2].x0 - 300) - abs(right_bound[2].x0 -300)) << endl;
  uhid_key_event(pfds.fd, USB_KEY_W, 30);

  if (( abs(abs(left_bound[2].x0 - 300) - abs(right_bound[2].x0 -300))) < 30){
    cout << "go straight" << endl;
    uhid_key_event(pfds.fd, USB_KEY_W, 10);
  }
  
  else{
    
    if ( ((abs(left_bound[2].x0 - 300) - abs(right_bound[2].x0 -300)) < 0) && (abs(left_bound[2].x0 - 300) - abs(right_bound[2].x0 -300)) > -60)
      {
	cout << " right turn" <<endl;
	uhid_key_event(pfds.fd, USB_KEY_D, 200);
	uhid_key_event(pfds.fd, USB_KEY_S, 10);	
      }
    
    else if((( abs(left_bound[2].x0 - 300) - abs(right_bound[2].x0 -300)) > 0) && (( abs(left_bound[2].x0 - 300) - abs(right_bound[2].x0 -300)) < 60))
      {
	cout << "left turn" <<endl;
	uhid_key_event(pfds.fd, USB_KEY_A, 200);
	uhid_key_event(pfds.fd, USB_KEY_S, 10);	
      }

    else if(( abs(left_bound[2].x0 - 300) - abs(right_bound[2].x0 -300) < -60) )
      {
	uhid_key_event(pfds.fd, USB_KEY_D, 600);
	uhid_key_event(pfds.fd, USB_KEY_S, 20);
      }
    
    else if(( abs(left_bound[2].x0 - 300) - abs(right_bound[2].x0 -300) > 60) )
      {
	uhid_key_event(pfds.fd, USB_KEY_A, 600);
	uhid_key_event(pfds.fd, USB_KEY_S, 20);
      }
  }
  
}

    
int main(int argc, char* argv[])
{
  
  cv::String window_name = "My First Video";
  Display *disp = XOpenDisplay (NULL);
  uhid_init();
  namedWindow(window_name, WINDOW_NORMAL); 
  ScreenShot screen("torcs", 0,0,640,480);

  while (true)
    {
      Mat frame, a;
      int merged_lines_size;
      road roadanalysis;
      Mat img;
      single_line merged_lines[500];
      selfdrive test1;

      screen(a);
      resize(a, img, Size(600,400));
 
      roadanalysis.detect_road(img, merged_lines);

      test1.self_drive_control(roadanalysis.left_bounds, roadanalysis.right_bounds);


      waitKey(20);
    }
  uhid_stop(pfds.fd);
  return 0;

}

