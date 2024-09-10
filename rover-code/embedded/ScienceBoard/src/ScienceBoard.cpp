#include "../include/ScienceBoard.h"

ScienceBoard::ScienceBoard()
{

}

ScienceBoard::~ScienceBoard()
{
    // delete this->soilCollector;
    // delete this->soilTransferer;
    // delete this->fluorometer;
}
void ScienceBoard::UpdateSubsystems(void)
{
    this->soilCollector.update();
    this->soilTransferer.update();
    this->flurometer.update();
}