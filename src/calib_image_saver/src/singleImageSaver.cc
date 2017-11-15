#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../include/calib_image_saver/chessboard/Chessboard.h"

ros::Subscriber image_sub;

std::string image_path;
bool is_use_OpenCV = false;
bool is_show       = false;
bool is_save_data  = false;
std::string data_path;
std::string image_name = "IMG_";
cv::Size boardSize;

cv::Size image_size;
int image_count       = 0;
bool is_first_run     = true;
bool is_get_chessbord = false;
ros::Time time_last, time_now;
int max_freq = 10;
cv::Mat image_in, image_show;
cv::Mat DistributedImage;
std::vector< std::vector< cv::Point2f > > total_image_points;

void
showImage( cv::Mat& image, cv::Mat& _DistributedImage )
{
    if ( image.channels( ) == 1 )
        cv::cvtColor( image, image_show, CV_GRAY2RGB );
    else
        image_show = image;

    cv::Mat imgROI = _DistributedImage( cv::Rect( image.cols, 0, image.cols, image.rows ) );
    image_show.copyTo( imgROI );
    cv::imshow( "DistributedImage", _DistributedImage );
    cv::waitKey( 1000 / max_freq );
}

void
drawChessBoard( cv::Mat& image_input, cv::Mat& _DistributedImage, const std::vector< cv::Point2f >& imagePoints )
{
    int drawShiftBits  = 4;
    int drawMultiplier = 1 << drawShiftBits;

    cv::Scalar yellow( 0, 255, 255 );
    cv::Scalar green( 0, 255, 0 );

    cv::Mat& image = image_input;

    if ( image.channels( ) == 1 )
        cv::cvtColor( image, image, CV_GRAY2RGB );

    for ( size_t j = 0; j < imagePoints.size( ); ++j )
    {
        cv::Point2f pObs = imagePoints.at( j );

        // green points is the observed points
        cv::circle( image, cv::Point( cvRound( pObs.x * drawMultiplier ), cvRound( pObs.y * drawMultiplier ) ), 5,
                    green, 2, CV_AA, drawShiftBits );

        // yellow points is the observed points
        cv::circle( _DistributedImage, cv::Point( cvRound( pObs.x * drawMultiplier ), cvRound( pObs.y * drawMultiplier ) ),
                    5, yellow, 2, CV_AA, drawShiftBits );
    }

    cv::line( _DistributedImage, imagePoints.at( 0 ), imagePoints.at( boardSize.width - 1 ), green, 1 );
    cv::line( _DistributedImage, imagePoints.at( boardSize.width * ( boardSize.height - 1 ) ), imagePoints.at( 0 ), green, 1 );
    cv::line( _DistributedImage, imagePoints.at( boardSize.width * ( boardSize.height - 1 ) ),
              imagePoints.at( boardSize.width * boardSize.height - 1 ), green, 1 );
    cv::line( _DistributedImage, imagePoints.at( boardSize.width * boardSize.height - 1 ),
              imagePoints.at( boardSize.width - 1 ), green, 1 );
}

void
callback_0( const sensor_msgs::Image::ConstPtr& img )
{
    image_in = cv_bridge::toCvCopy( img, "mono8" )->image;
    time_now = img->header.stamp;

    if ( is_first_run )
    {
        time_last         = img->header.stamp;
        image_size.height = img->height;
        image_size.width  = img->width;
        cv::Mat DistributedImage_tmp( cv::Size( image_size.width * 2, image_size.height ), CV_8UC3, cv::Scalar( 0 ) );

        DistributedImage_tmp.copyTo( DistributedImage );
        is_first_run = false;

        if ( is_show )
            cv::namedWindow( "DistributedImage", CV_WINDOW_NORMAL );
    }

    if ( is_show )
        showImage( image_in, DistributedImage );
}

void
process( )
{
    if ( is_first_run )
        return;

    cv::Mat& image_input = image_in;

    camera_model::Chessboard chessboard( boardSize, image_input );

    chessboard.findCorners( is_use_OpenCV );

    if ( chessboard.cornersFound( ) )
    {
        std::stringstream ss_num;

        ss_num << image_count;
        std::string image_file = image_path + "/" + image_name + ss_num.str( ) + ".jpg";
        std::cout << "#[INFO] Get chessboard image: " << image_name + ss_num.str( ) + ".jpg" << std::endl;

        cv::imwrite( image_file, image_input );

        ++image_count;
        total_image_points.push_back( chessboard.getCorners( ) );

        if ( is_show )
        {
            drawChessBoard( image_input, DistributedImage, total_image_points.back( ) );
            showImage( image_input, DistributedImage );
        }
        is_get_chessbord = true;
    }
    else
    {
        std::cout << "#[ERROR] Get no chessboard image." << std::endl;
    }
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "singleImageSaver" );
    ros::NodeHandle n( "~" );

    n.getParam( "rate", max_freq );
    n.getParam( "image_path", image_path );
    n.getParam( "board_width", boardSize.width );
    n.getParam( "board_height", boardSize.height );
    n.getParam( "is_use_OpenCV", is_use_OpenCV );
    n.getParam( "is_show", is_show );
    n.getParam( "is_save_data", is_save_data );
    n.getParam( "data_path", data_path );
    n.getParam( "image_name", image_name );

    if ( boardSize.height <= 1 || boardSize.width <= 1 )
    {
        std::cout << "#[ERROR] Error with input chessbopard Size." << std::endl;
        return 0;
    }

    image_sub = n.subscribe< sensor_msgs::Image >( "/image_input", 3, callback_0, ros::TransportHints( ).tcpNoDelay( ) );

    ros::Rate loop( max_freq );
    while ( ros::ok( ) )
    {
        process( );
        ros::spinOnce( );
        loop.sleep( );
    }
    //   ros::spin( );

    cv::imwrite( image_path + "/IMG_" + "Distributed.jpg", DistributedImage );
    std::cout << "#[INFO] Get chessboard iamges: " << image_count << std::endl;

    return 0;
}
