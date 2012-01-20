//Template wrapper class for access to IplImage* from cvopen library
// access format: [typedef class] [name_interface](IplImage_input); typedefs listed below
template<class T> class Image
{
  private:
  IplImage* imgp;
  public:
  Image(IplImage* img=0) {imgp=img;}
  ~Image(){imgp=0;}
  void operator=(IplImage* img) {imgp=img;}
  inline T* operator[](const int rowIndx) {
    return ((T *)(imgp->imageData + rowIndx*imgp->widthStep));}
};
// part of template wrapper
typedef struct{
  unsigned char b,g,r;
} RgbPixel;
// part of template wrapper
typedef struct{
  float b,g,r;
} RgbPixelFloat;
//Typedefs for template wrapper. Use these to access different types of pictures
typedef Image<RgbPixel>       RgbImage;
typedef Image<RgbPixelFloat>  RgbImageFloat;
typedef Image<unsigned char>  BwImage;
typedef Image<float>          BwImageFloat;