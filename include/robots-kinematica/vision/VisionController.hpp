#include "ObjectDetector.hpp"
#include <ros/ros.h>
#include <regex>
#include <iostream>
#include <fstream>
#include <memory>
#include <thread>
#include <atomic>


class VisionController
{
  public:
    /**
     * @brief Construct a new VisionController object
     * 
     * @param aInputData string that contains the data of a batch file.
     */
    VisionController();

    /**
     * @brief Destroy the VisionController object
     * 
     */
    ~VisionController();

    /**
     * @brief Splits the string in two seperate words based on the format.
     * 
     * @param str string contains a single commando's of the user or batch file
     */
    void splitString(std::string str);

    /**
     * @brief splits batch file line by line based of the regex.
     * 
     */
    void splitAndStoreLinesBasedOnRegex();

    /**
     * @brief Checks the string values based of the static figures and colors.
     * 
     * @param figure string that contains a figure. 
     * @param color string that contains a color.
     */
    void checkStringValues(const std::string &figure, const std::string &color);

    /**
     * @brief The main while loop of the VisionController.
     * 
     * @param boolean for the mode of the VisionController.
     * 0 for interactive modus.
     * 1 for batch modus.
     */
    void VisionControllerLoop();

    /**
     * @brief Set the Filtered Frame object
     * 
     * @param aFiltereFrame 
     */
    void setFilteredFrame(const cv::Mat &aFiltereFrame);


    /**
     * @brief Function that reads the user input.
     * 
     */
    void readCin();
    
    void checkString(const std::string & str);

    void findColorAndShape(const std::string& inputColor, const std::string& inputFigure);


  private:
    cv::VideoCapture cap;
    cv::Mat frame;
    cv::Mat filteredFrame;
    std::string inputData;
    char input[100];
    bool inputFromUser;
    bool m_foundShapeObject = false;
    std::shared_ptr<ObjectDetector> detecter;
    std::shared_ptr<ColorObject> m_colorObject;

};