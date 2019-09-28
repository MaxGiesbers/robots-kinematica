#include "robots-kinematica/vision/VisionController.hpp"
#include <time.h>
namespace
{
    const std::vector<std::string> COLORS = {"groen", "blauw", "rood", "wit", "zwart", "geel"};
    const std::vector<std::string> FIGURES = {"vierkant", "rechthoek", "cirkel", "halve cirkel", "driehoek"};
    const uint8_t ROS_LOOP_RATE = 20;

}

VisionController::VisionController() 
{
}

VisionController::~VisionController()
{
}

void VisionController::checkString(const std::string &str)
{
    std::stringstream stream(str);
    unsigned int NumberOfWords = (int)std::distance(std::istream_iterator<std::string>(stream),
                                                    std::istream_iterator<std::string>());

    if (NumberOfWords < 2)
    {
        if (str.compare("start") == 0)
        {
            return;
        }
        else if (str.compare("exit") == 0)
        {
            std::cout << "Exit is ingevoerd het programma is afgesloten." << std::endl;
        }
        //error = true;
    }
}

void VisionController::splitString(std::string str)
{
    size_t index = str.find_last_of(" ");
    checkString(str);
    if (index != std::string::npos)
    {
        //Last substring for the color.
        std::string color = str.substr(index + 1, str.length());
        //remove whitespaces
        color.erase(color.find_last_not_of(" \t\n\r\f\v") + 1);
        //remove color.
        str.erase(index + 1, str.length());
        //remaining the figure
        std::string figure = str;
        //remove whitespaces
        figure.erase(figure.find_last_not_of(" \t\n\r\f\v") + 1);
        findColorAndShape(color,figure);
    }
}


void VisionController::findColorAndShape(const std::string& inputColor, const std::string& inputFigure)
{
    std::vector<std::string>::const_iterator color = std::find_if(COLORS.begin(),
    COLORS.end(), 
    [&](const auto& color){ return color.compare(inputColor) == 0; });

    std::vector<std::string>::const_iterator figure = std::find_if(FIGURES.begin(),
    FIGURES.end(), 
    [&](const auto& figure){ return figure.compare(inputFigure) == 0; });
    
    if (color != std::end(COLORS) && figure != std::end(FIGURES)  ) 
    {
        m_colorObject = std::make_shared<ColorObject>(*color, *figure);
        m_foundShapeObject = true;
    } 
    else 
    {
        m_foundShapeObject = false;
    }
}

void VisionController::readCin()
{
    std::cout << "Voer een vorm en een kleur in met als format: [vorm][whitespace][kleur]" << std::endl;

    std::string input = "";
    ros::Rate rate(ROS_LOOP_RATE);

    while (ros::ok())
    {
        if(!m_foundShapeObject)
        {
            input = "";
            //clean cin input
            std::cin >> std::ws;
            std::getline(std::cin, input);
            splitString(input);
            inputFromUser = true;
        }
    }
}

void VisionController::VisionControllerLoop()
{
    cap.open(0);
    //runs a thread for userInput

    while (true)
    {
        cap >> frame;
        imshow("live", frame);
        cv::waitKey(30);
        if (m_foundShapeObject) //Batch modus.
        {
            filteredFrame = frame.clone();
            detecter = std::make_shared<ObjectDetector>(frame, filteredFrame);
            cv::GaussianBlur(filteredFrame, filteredFrame, cv::Size(9, 9), 0, 0);
            cv::erode(filteredFrame, filteredFrame, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
            cv::dilate(filteredFrame, filteredFrame, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
            
            detecter->filterColor(m_colorObject);
            detecter->findShape(m_colorObject);
            
            std::cout << "Voer een vorm en een kleur in met als format: [vorm][whitespace][kleur]" 
                << std::endl;
            
            inputFromUser = false;
            imshow("object detector", frame);
            
        }
    }
    cv::waitKey(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision");
    ros::Time::init();
    VisionController visionController = VisionController();
    std::thread userInputThread(&VisionController::readCin, VisionController());

    visionController.VisionControllerLoop();
    userInputThread.join();
    
    return 0;
}