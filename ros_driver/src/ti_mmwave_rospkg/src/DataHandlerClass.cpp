#include <DataHandlerClass.h>

DataUARTHandler::DataUARTHandler(ros::NodeHandle* nh, char* myRadarBin) : currentBufp(&pingPongBuffers[0]) , nextBufp(&pingPongBuffers[1]) {
    marker_pub = nh->advertise<visualization_msgs::Marker>("/ti_mmwave/radar_scan_markers", 100);
    maxAllowedElevationAngleDeg = 90; // Use max angle if none specified
    maxAllowedAzimuthAngleDeg = 90; // Use max angle if none specified

    setRadarBin(myRadarBin);

    if (radarBin == BIN_XWR18XX_MRR)
    {
        // Wait some time
        ros::Duration(1).sleep();

        // Plublisher for rviz MRR detection
        DataUARTHandler_pub = nh->advertise<sensor_msgs::PointCloud2>("/ti_mmwave/radar_detection_mrr_rviz", 100);

        // Plublisher for message MRR detection
        radar_scan_pub = nh->advertise<radar_msgs::RadarDetectionArray>("/ti_mmwave/radar_detection_mrr", 100);

        // Plublisher for rviz USRR detection
        DataUARTHandler_pub_1 = nh->advertise<sensor_msgs::PointCloud2>("/ti_mmwave/radar_detection_usrr_rviz", 100);

        // Plublisher for message USRR detection
        radar_scan_pub_1 = nh->advertise<radar_msgs::RadarDetectionArray>("/ti_mmwave/radar_detection_usrr", 100);

        // Publisher for rviz MRR track
        DataUARTHandler_pub_2 = nh->advertise<visualization_msgs::MarkerArray>("/ti_mmwave/radar_track_mrr_rviz", 100);

        // Plublisher for message MRR track
        radar_scan_pub_2 = nh->advertise<radar_msgs::RadarTrackArray>("/ti_mmwave/radar_track_mrr", 100);

        // Publisher for rviz USRR track
        DataUARTHandler_pub_3 = nh->advertise<visualization_msgs::MarkerArray>("/ti_mmwave/radar_track_usrr_rviz", 100);

        // Plublisher for message USRR track
        radar_scan_pub_3 = nh->advertise<radar_msgs::RadarTrackArray>("/ti_mmwave/radar_track_usrr", 100);

        // Parameters obtained from mrr_18xx_mss -> common -> cfg.c:
        // Cfg_ProfileCfgInitParams, Cfg_AdvFrameCfgInitParams

        // MRR120 profile configuration
        float startFreq = 76.01;
        float idleTime = 5.0;
        float adcStartTime = 4.8;
        float rampEndTime = 60.0;
        float freqSlopeConst = 4.0;
        float numAdcSamples = 256;
        float digOutSampleRate = 4652;
        float rxGain = 44;

        // MRR120 frame configuration
        float chirpStartIdx = 0;
        float chirpEndIdx = 255;
        float numLoops = 1;
        float framePeriodicity = 30;

        // MRR120 parameters
        nr = numAdcSamples;
        nd = numLoops;
        ntx = chirpEndIdx - chirpStartIdx + 1;
        fs = digOutSampleRate * 1e3;

        // MRR120 intermediate parameters
        float kf = freqSlopeConst * 1e12;
        float adc_duration = nr / fs;
        float c0 = 299792458;

        // MRR120 parameters
        fc = startFreq * 1e9 + kf * (adcStartTime * 1e-6 + adc_duration / 2);
        BW = adc_duration * kf;
        PRI = (idleTime + rampEndTime) * 1e-6;
        tfr = framePeriodicity * 1e-3;
        vrange = c0 / (2 * BW);
        max_range = nr * vrange;
        max_vel = c0 / (2 * fc * PRI) / ntx;
        vvel = max_vel / nd;

        // USRR30 profile configuration
        float startFreq_1 = 77.01;
        float idleTime_1 = 5.0;
        float adcStartTime_1 = 3.0;
        float rampEndTime_1 = 44.0;
        float freqSlopeConst_1 = 56.25;
        float numAdcSamples_1 = 512;
        float digOutSampleRate_1 = 12500;
        float rxGain_1 = 30;

        // USRR30 frame configuration
        float chirpStartIdx_1 = 256;
        float chirpEndIdx_1 = 258;
        float numLoops_1 = 32;
        float framePeriodicity_1 = 30;

        // USRR30 parameters
        nr_1 = numAdcSamples_1;
        nd_1 = numLoops_1;
        ntx_1 = chirpEndIdx_1 - chirpStartIdx_1 + 1;
        fs_1 = digOutSampleRate_1 * 1e3;

        // USRR30 intermediate parameters
        float kf_1 = freqSlopeConst_1 * 1e12;
        float adc_duration_1 = nr_1 / fs_1;
        float c0_1 = 299792458;

        // USRR30 parameters
        fc_1 = startFreq_1 * 1e9 + kf_1 * (adcStartTime_1 * 1e-6 + adc_duration_1 / 2);
        BW_1 = adc_duration_1 * kf_1;
        PRI_1 = (idleTime_1 + rampEndTime_1) * 1e-6;
        tfr_1 = framePeriodicity_1 * 1e-3;
        vrange_1 = c0_1 / (2 * BW_1);
        max_range_1 = nr_1 * vrange_1;
        max_vel_1 = c0_1 / (2 * fc_1 * PRI_1) / ntx_1;
        vvel_1 = max_vel_1 / nd_1;

        ROS_INFO("\n\n==============================\nList of MRR parameters\n==============================\nNumber of range samples: %d\nNumber of chirps: %d\nf_s: %.3f MHz\nf_c: %.3f GHz\nBandwidth: %.3f MHz\nPRI: %.3f us\nFrame time: %.3f ms\nMax range: %.3f m\nRange resolution: %.3f m\nMax Doppler: +-%.3f m/s\nDoppler resolution: %.3f m/s\n==============================\n", \
            nr, nd, fs/1e6, fc/1e9, BW/1e6, PRI*1e6, tfr*1e3, max_range, vrange, max_vel/2, vvel);
        ROS_INFO("\n\n==============================\nList of USRR parameters\n==============================\nNumber of range samples: %d\nNumber of chirps: %d\nf_s: %.3f MHz\nf_c: %.3f GHz\nBandwidth: %.3f MHz\nPRI: %.3f us\nFrame time: %.3f ms\nMax range: %.3f m\nRange resolution: %.3f m\nMax Doppler: +-%.3f m/s\nDoppler resolution: %.3f m/s\n==============================\n", \
            nr_1, nd_1, fs_1/1e6, fc_1/1e9, BW_1/1e6, PRI_1*1e6, tfr_1*1e3, max_range_1, vrange_1, max_vel_1/2, vvel_1);
    }
    else
    {
        DataUARTHandler_pub = nh->advertise<sensor_msgs::PointCloud2>("/ti_mmwave/radar_scan_pcl", 100);
        radar_scan_pub = nh->advertise<ti_mmwave_rospkg::RadarScan>("/ti_mmwave/radar_scan", 100);
        
        // Wait for parameters
        while(!nh->hasParam("/ti_mmwave/doppler_vel_resolution")){}

        nh->getParam("/ti_mmwave/numAdcSamples", nr);
        nh->getParam("/ti_mmwave/numLoops", nd);
        nh->getParam("/ti_mmwave/num_TX", ntx);
        nh->getParam("/ti_mmwave/f_s", fs);
        nh->getParam("/ti_mmwave/f_c", fc);
        nh->getParam("/ti_mmwave/BW", BW);
        nh->getParam("/ti_mmwave/PRI", PRI);
        nh->getParam("/ti_mmwave/t_fr", tfr);
        nh->getParam("/ti_mmwave/max_range", max_range);
        nh->getParam("/ti_mmwave/range_resolution", vrange);
        nh->getParam("/ti_mmwave/max_doppler_vel", max_vel);
        nh->getParam("/ti_mmwave/doppler_vel_resolution", vvel);

        ROS_INFO("\n\n==============================\nList of parameters\n==============================\nNumber of range samples: %d\nNumber of chirps: %d\nf_s: %.3f MHz\nf_c: %.3f GHz\nBandwidth: %.3f MHz\nPRI: %.3f us\nFrame time: %.3f ms\nMax range: %.3f m\nRange resolution: %.3f m\nMax Doppler: +-%.3f m/s\nDoppler resolution: %.3f m/s\n==============================\n", \
            nr, nd, fs/1e6, fc/1e9, BW/1e6, PRI*1e6, tfr*1e3, max_range, vrange, max_vel/2, vvel);
    }
}

void DataUARTHandler::setFrameID(char* myFrameID)
{
    frameID = myFrameID;
}

/*Implementation of setUARTPort*/
void DataUARTHandler::setUARTPort(char* mySerialPort)
{
    dataSerialPort = mySerialPort;
}

/*Implementation of setBaudRate*/
void DataUARTHandler::setBaudRate(int myBaudRate)
{
    dataBaudRate = myBaudRate;
}

/*Implementation of setMaxAllowedElevationAngleDeg*/
void DataUARTHandler::setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg)
{
    maxAllowedElevationAngleDeg = myMaxAllowedElevationAngleDeg;
}

/*Implementation of setMaxAllowedAzimuthAngleDeg*/
void DataUARTHandler::setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg)
{
    maxAllowedAzimuthAngleDeg = myMaxAllowedAzimuthAngleDeg;
}
    
/*Implementation of setRadarBin*/
void DataUARTHandler::setRadarBin(char* myRadarBin)
{
    if (strcmp(myRadarBin, "xwr18xx_mrr_demo.bin") == 0)
    {
        radarBin = BIN_XWR18XX_MRR;
    }
    else
    {
        radarBin = BIN_GENERIC;
    }
}

/*Implementation of readIncomingData*/
void *DataUARTHandler::readIncomingData(void)
{
    
    int firstPacketReady = 0;
    uint8_t last8Bytes[8] = {0};
    
    /*Open UART Port and error checking*/
    serial::Serial mySerialObject("", dataBaudRate, serial::Timeout::simpleTimeout(100));
    mySerialObject.setPort(dataSerialPort);
    try
    {
        mySerialObject.open();
    } catch (std::exception &e1) {
        ROS_INFO("DataUARTHandler Read Thread: Failed to open Data serial port with error: %s", e1.what());
        ROS_INFO("DataUARTHandler Read Thread: Waiting 20 seconds before trying again...");
        try
        {
            // Wait 20 seconds and try to open serial port again
            ros::Duration(20).sleep();
            mySerialObject.open();
        } catch (std::exception &e2) {
            ROS_ERROR("DataUARTHandler Read Thread: Failed second time to open Data serial port, error: %s", e1.what());
            ROS_ERROR("DataUARTHandler Read Thread: Port could not be opened. Port is \"%s\" and baud rate is %d", dataSerialPort, dataBaudRate);
            pthread_exit(NULL);
        }
    }
    
    if(mySerialObject.isOpen())
        ROS_INFO("DataUARTHandler Read Thread: Port is open");
    else
        ROS_ERROR("DataUARTHandler Read Thread: Port could not be opened");
    
    /*Quick magicWord check to synchronize program with data Stream*/
    while(!isMagicWord(last8Bytes))
    {

        last8Bytes[0] = last8Bytes[1];
        last8Bytes[1] = last8Bytes[2];
        last8Bytes[2] = last8Bytes[3];
        last8Bytes[3] = last8Bytes[4];
        last8Bytes[4] = last8Bytes[5];
        last8Bytes[5] = last8Bytes[6];
        last8Bytes[6] = last8Bytes[7];
        mySerialObject.read(&last8Bytes[7], 1);
        
    }
    
    /*Lock nextBufp before entering main loop*/
    pthread_mutex_lock(&nextBufp_mutex);
    
    while(ros::ok())
    {
        /*Start reading UART data and writing to buffer while also checking for magicWord*/
        last8Bytes[0] = last8Bytes[1];
        last8Bytes[1] = last8Bytes[2];
        last8Bytes[2] = last8Bytes[3];
        last8Bytes[3] = last8Bytes[4];
        last8Bytes[4] = last8Bytes[5];
        last8Bytes[5] = last8Bytes[6];
        last8Bytes[6] = last8Bytes[7];
        mySerialObject.read(&last8Bytes[7], 1);
        
        nextBufp->push_back( last8Bytes[7] );  //push byte onto buffer
        
        //ROS_INFO("DataUARTHandler Read Thread: last8bytes = %02x%02x %02x%02x %02x%02x %02x%02x",  last8Bytes[7], last8Bytes[6], last8Bytes[5], last8Bytes[4], last8Bytes[3], last8Bytes[2], last8Bytes[1], last8Bytes[0]);
        
        /*If a magicWord is found wait for sorting to finish and switch buffers*/
        if( isMagicWord(last8Bytes) )
        {
            //ROS_INFO("Found magic word");
        
            /*Lock countSync Mutex while unlocking nextBufp so that the swap thread can use it*/
            pthread_mutex_lock(&countSync_mutex);
            pthread_mutex_unlock(&nextBufp_mutex);
            
            /*increment countSync*/
            countSync++;
            
            /*If this is the first packet to be found, increment countSync again since Sort thread is not reading data yet*/
            if(firstPacketReady == 0)
            {
                countSync++;
                firstPacketReady = 1;
            }
            
            /*Signal Swap Thread to run if countSync has reached its max value*/
            if(countSync == COUNT_SYNC_MAX)
            {
                pthread_cond_signal(&countSync_max_cv);
            }
            
            /*Wait for the Swap thread to finish swapping pointers and signal us to continue*/
            pthread_cond_wait(&read_go_cv, &countSync_mutex);
            
            /*Unlock countSync so that Swap Thread can use it*/
            pthread_mutex_unlock(&countSync_mutex);
            pthread_mutex_lock(&nextBufp_mutex);
            
            nextBufp->clear();
            memset(last8Bytes, 0, sizeof(last8Bytes));
              
        }
      
    }
    
    
    mySerialObject.close();
    
    pthread_exit(NULL);
}


int DataUARTHandler::isMagicWord(uint8_t last8Bytes[8])
{
    int val = 0, i = 0, j = 0;
    
    for(i = 0; i < 8 ; i++)
    {
    
       if( last8Bytes[i] == magicWord[i])
       {
          j++;
       }
    
    }
    
    if( j == 8)
    {
       val = 1;
    }
    
    return val;  
}

void *DataUARTHandler::syncedBufferSwap(void)
{
    while(ros::ok())
    {
        pthread_mutex_lock(&countSync_mutex);
    
        while(countSync < COUNT_SYNC_MAX)
        {
            pthread_cond_wait(&countSync_max_cv, &countSync_mutex);
            
            pthread_mutex_lock(&currentBufp_mutex);
            pthread_mutex_lock(&nextBufp_mutex);
            
            std::vector<uint8_t>* tempBufp = currentBufp;
        
            this->currentBufp = this->nextBufp;
            
            this->nextBufp = tempBufp;
            
            pthread_mutex_unlock(&currentBufp_mutex);
            pthread_mutex_unlock(&nextBufp_mutex);
            
            countSync = 0;
            
            pthread_cond_signal(&sort_go_cv);
            pthread_cond_signal(&read_go_cv);
            
        }
    
        pthread_mutex_unlock(&countSync_mutex);

    }

    pthread_exit(NULL);
    
}

void *DataUARTHandler::sortIncomingData( void )
{
    MmwDemo_Output_TLV_Types tlvType = MMWDEMO_OUTPUT_MSG_NULL;
    uint32_t tlvLen = 0;
    uint32_t headerSize;
    unsigned int currentDatap = 0;
    SorterState sorterState = READ_HEADER;
    int i = 0, tlvCount = 0, offset = 0;
    int j = 0;
    float maxElevationAngleRatioSquared;
    float maxAzimuthAngleRatio;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> RScan(new pcl::PointCloud<pcl::PointXYZI>);
    ti_mmwave_rospkg::RadarScan radarscan;
    
    // Detection (object) point for rviz
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> RScanObjectRviz(new pcl::PointCloud<pcl::PointXYZI>);

    // Detection (object) standard message
    boost::shared_ptr<radar_msgs::RadarDetectionArray> RScanObjectStd(new radar_msgs::RadarDetectionArray);

    // USRR track (cluster) marker for rviz
    boost::shared_ptr<visualization_msgs::MarkerArray> RScanClusterRviz(new visualization_msgs::MarkerArray);

    // USRR track (cluster) standard message
    boost::shared_ptr<radar_msgs::RadarTrackArray> RScanClusterStd(new radar_msgs::RadarTrackArray);

    // MRR track (tracked object) marker for rviz
    boost::shared_ptr<visualization_msgs::MarkerArray> RScanTrackRviz(new visualization_msgs::MarkerArray);

    // MRR track (tracked object) standard message
    boost::shared_ptr<radar_msgs::RadarTrackArray> RScanTrackStd(new radar_msgs::RadarTrackArray);

    //wait for first packet to arrive
    pthread_mutex_lock(&countSync_mutex);
    pthread_cond_wait(&sort_go_cv, &countSync_mutex);
    pthread_mutex_unlock(&countSync_mutex);
    
    pthread_mutex_lock(&currentBufp_mutex);
    
    while(ros::ok())
    {
        
        switch(sorterState)
        {
            
        case READ_HEADER:
            // ROS_INFO("DataUARTHandler Sort Thread : case READ_HEADER");
            
            //init variables
            mmwData.numObjOut = 0;
            mmwData.numClusterOut = 0;
            mmwData.numTrackOut = 0;

            //make sure packet has at least first three fields (12 bytes) before we read them (does not include magicWord since it was already removed)
            if(currentBufp->size() < 12)
            {
               sorterState = SWAP_BUFFERS;
               break;
            }
            
            //get version (4 bytes)
            memcpy( &mmwData.header.version, &currentBufp->at(currentDatap), sizeof(mmwData.header.version));
            currentDatap += ( sizeof(mmwData.header.version) );
            
            //get totalPacketLen (4 bytes)
            memcpy( &mmwData.header.totalPacketLen, &currentBufp->at(currentDatap), sizeof(mmwData.header.totalPacketLen));
            currentDatap += ( sizeof(mmwData.header.totalPacketLen) );
            
            //get platform (4 bytes)
            memcpy( &mmwData.header.platform, &currentBufp->at(currentDatap), sizeof(mmwData.header.platform));
            currentDatap += ( sizeof(mmwData.header.platform) );      
            
            //if packet doesn't have correct header size (which is based on platform), throw it away
            //  (does not include magicWord since it was already removed)
            if ((mmwData.header.platform & 0xFFFF) == 0x1443)  // platform is xWR1443)
            {
               headerSize = 7 * 4;  // xWR1443 SDK demo header does not have subFrameNumber field
            }
            else
            {
               headerSize = 8 * 4;  // header includes subFrameNumber field
            }
            if(currentBufp->size() < headerSize) {
                sorterState = SWAP_BUFFERS;
                break;
            }
            
            //get frameNumber (4 bytes)
            memcpy( &mmwData.header.frameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.frameNumber));
            currentDatap += ( sizeof(mmwData.header.frameNumber) );
            
            //get timeCpuCycles (4 bytes)
            memcpy( &mmwData.header.timeCpuCycles, &currentBufp->at(currentDatap), sizeof(mmwData.header.timeCpuCycles));
            currentDatap += ( sizeof(mmwData.header.timeCpuCycles) );
            
            //get numDetectedObj (4 bytes)
            memcpy( &mmwData.header.numDetectedObj, &currentBufp->at(currentDatap), sizeof(mmwData.header.numDetectedObj));
            currentDatap += ( sizeof(mmwData.header.numDetectedObj) );
            
            //get numTLVs (4 bytes)
            memcpy( &mmwData.header.numTLVs, &currentBufp->at(currentDatap), sizeof(mmwData.header.numTLVs));
            currentDatap += ( sizeof(mmwData.header.numTLVs) );
            
            //get subFrameNumber (4 bytes) (not used for XWR1443)
            if((mmwData.header.platform & 0xFFFF) != 0x1443) {
               memcpy( &mmwData.header.subFrameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.subFrameNumber));
               currentDatap += ( sizeof(mmwData.header.subFrameNumber) );
            }

            //if packet lengths do not match, throw it away
            if(mmwData.header.totalPacketLen == currentBufp->size() )
            {
               sorterState = CHECK_TLV_TYPE;
            }
            else sorterState = SWAP_BUFFERS;

            break;
            
        case READ_OBJ_STRUCT:
            
            // CHECK_TLV_TYPE code has already read tlvType and tlvLen

            i = 0;
            offset = 0;
            
            if (((mmwData.header.version >> 24) & 0xFF) < 3)  // SDK version is older than 3.x
            {
                //get number of objects
                memcpy( &mmwData.numObjOut, &currentBufp->at(currentDatap), sizeof(mmwData.numObjOut));
                currentDatap += ( sizeof(mmwData.numObjOut) );
            
                //get xyzQFormat
                memcpy( &mmwData.xyzQFormat, &currentBufp->at(currentDatap), sizeof(mmwData.xyzQFormat));
                currentDatap += ( sizeof(mmwData.xyzQFormat) );
            }
            else  // SDK version is at least 3.x
            {
                mmwData.numObjOut = mmwData.header.numDetectedObj;
            }
            
            // RScan->header.seq = 0;
            // RScan->header.stamp = (uint64_t)(ros::Time::now());
            // RScan->header.stamp = (uint32_t) mmwData.header.timeCpuCycles;
            RScan->header.frame_id = frameID;
            RScan->height = 1;
            RScan->width = mmwData.numObjOut;
            RScan->is_dense = 1;
            RScan->points.resize(RScan->width * RScan->height);
            
            // Calculate ratios for max desired elevation and azimuth angles
            if ((maxAllowedElevationAngleDeg >= 0) && (maxAllowedElevationAngleDeg < 90)) {
                maxElevationAngleRatioSquared = tan(maxAllowedElevationAngleDeg * M_PI / 180.0);
                maxElevationAngleRatioSquared = maxElevationAngleRatioSquared * maxElevationAngleRatioSquared;
            } else maxElevationAngleRatioSquared = -1;
            if ((maxAllowedAzimuthAngleDeg >= 0) && (maxAllowedAzimuthAngleDeg < 90)) maxAzimuthAngleRatio = tan(maxAllowedAzimuthAngleDeg * M_PI / 180.0);
            else maxAzimuthAngleRatio = -1;

            //ROS_INFO("maxElevationAngleRatioSquared = %f", maxElevationAngleRatioSquared);
            //ROS_INFO("maxAzimuthAngleRatio = %f", maxAzimuthAngleRatio);
            //ROS_INFO("mmwData.numObjOut before = %d", mmwData.numObjOut);

            // Populate pointcloud
            while( i < mmwData.numObjOut ) {
                if (((mmwData.header.version >> 24) & 0xFF) < 3) { // SDK version is older than 3.x
                    //get object range index
                    memcpy( &mmwData.objOut.rangeIdx, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.rangeIdx));
                    currentDatap += ( sizeof(mmwData.objOut.rangeIdx) );
                    
                    //get object doppler index
                    memcpy( &mmwData.objOut.dopplerIdx, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.dopplerIdx));
                    currentDatap += ( sizeof(mmwData.objOut.dopplerIdx) );
                    
                    //get object peak intensity value
                    memcpy( &mmwData.objOut.peakVal, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.peakVal));
                    currentDatap += ( sizeof(mmwData.objOut.peakVal) );
                    
                    //get object x-coordinate
                    memcpy( &mmwData.objOut.x, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.x));
                    currentDatap += ( sizeof(mmwData.objOut.x) );
                    
                    //get object y-coordinate
                    memcpy( &mmwData.objOut.y, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.y));
                    currentDatap += ( sizeof(mmwData.objOut.y) );
                    
                    //get object z-coordinate
                    memcpy( &mmwData.objOut.z, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.z));
                    currentDatap += ( sizeof(mmwData.objOut.z) );
                    
                    float temp[8];
                    
                    temp[0] = (float) mmwData.objOut.x;
                    temp[1] = (float) mmwData.objOut.y;
                    temp[2] = (float) mmwData.objOut.z;
                    temp[3] = (float) mmwData.objOut.dopplerIdx;

                    for (int j = 0; j < 4; j++) {
                        if (temp[j] > 32767) temp[j] -= 65536;
                        if (j < 3) temp[j] = temp[j] / pow(2 , mmwData.xyzQFormat);
                    }   
                    
                    temp[7] = temp[3] * vvel;

                    temp[4] = (float) mmwData.objOut.rangeIdx * vrange;
                    temp[5] = 10 * log10(mmwData.objOut.peakVal + 1);  // intensity
                    temp[6] = std::atan2(-temp[0], temp[1]) / M_PI * 180;
                    
                    uint16_t tmp = (uint16_t)(temp[3] + nd / 2);

                    // Map mmWave sensor coordinates to ROS coordinate system
                    RScan->points[i].x = temp[1];   // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
                    RScan->points[i].y = -temp[0];  // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
                    RScan->points[i].z = temp[2];   // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis
                    RScan->points[i].intensity = temp[5];
                    
                    radarscan.header.frame_id = frameID;
                    radarscan.header.stamp = ros::Time::now();

                    radarscan.point_id = i;
                    radarscan.x = temp[1];
                    radarscan.y = -temp[0];
                    radarscan.z = temp[2];
                    radarscan.range = temp[4];
                    radarscan.velocity = temp[7];
                    radarscan.doppler_bin = tmp;
                    radarscan.bearing = temp[6];
                    radarscan.intensity = temp[5];
                } else { // SDK version is 3.x+
                    //get object x-coordinate (meters)
                    memcpy( &mmwData.newObjOut.x, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.x));
                    currentDatap += ( sizeof(mmwData.newObjOut.x) );
                
                    //get object y-coordinate (meters)
                    memcpy( &mmwData.newObjOut.y, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.y));
                    currentDatap += ( sizeof(mmwData.newObjOut.y) );
                
                    //get object z-coordinate (meters)
                    memcpy( &mmwData.newObjOut.z, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.z));
                    currentDatap += ( sizeof(mmwData.newObjOut.z) );
                
                    //get object velocity (m/s)
                    memcpy( &mmwData.newObjOut.velocity, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.velocity));
                    currentDatap += ( sizeof(mmwData.newObjOut.velocity) );

                    // Map mmWave sensor coordinates to ROS coordinate system
                    RScan->points[i].x = mmwData.newObjOut.y;   // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
                    RScan->points[i].y = -mmwData.newObjOut.x;  // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
                    RScan->points[i].z = mmwData.newObjOut.z;   // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis

                    radarscan.header.frame_id = frameID;
                    radarscan.header.stamp = ros::Time::now();

                    radarscan.point_id = i;
                    radarscan.x = mmwData.newObjOut.y;
                    radarscan.y = -mmwData.newObjOut.x;
                    radarscan.z = mmwData.newObjOut.z;
                    // radarscan.range = temp[4];
                    radarscan.velocity = mmwData.newObjOut.velocity;
                    // radarscan.doppler_bin = tmp;
                    // radarscan.bearing = temp[6];
                    // radarscan.intensity = temp[5];
                    

                    // For SDK 3.x, intensity is replaced by snr in sideInfo and is parsed in the READ_SIDE_INFO code
                }

                if (((maxElevationAngleRatioSquared == -1) ||
                             (((RScan->points[i].z * RScan->points[i].z) / (RScan->points[i].x * RScan->points[i].x +
                                                                            RScan->points[i].y * RScan->points[i].y)
                              ) < maxElevationAngleRatioSquared)
                            ) &&
                            ((maxAzimuthAngleRatio == -1) || (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) &&
                                    (RScan->points[i].x != 0)
                           )
                {
                    radar_scan_pub.publish(radarscan);
                }
                i++;
            }

            sorterState = CHECK_TLV_TYPE;
            
            break;

        case READ_SIDE_INFO:

            // Make sure we already received and parsed detected obj list (READ_OBJ_STRUCT)
            if (mmwData.numObjOut > 0)
            {
                for (i = 0; i < mmwData.numObjOut; i++)
                {
                    //get snr (unit is 0.1 steps of dB)
                    memcpy( &mmwData.sideInfo.snr, &currentBufp->at(currentDatap), sizeof(mmwData.sideInfo.snr));
                    currentDatap += ( sizeof(mmwData.sideInfo.snr) );
                
                    //get noise (unit is 0.1 steps of dB)
                    memcpy( &mmwData.sideInfo.noise, &currentBufp->at(currentDatap), sizeof(mmwData.sideInfo.noise));
                    currentDatap += ( sizeof(mmwData.sideInfo.noise) );

                    RScan->points[i].intensity = (float) mmwData.sideInfo.snr / 10.0;   // Use snr for "intensity" field (divide by 10 since unit of snr is 0.1dB)
                }
            }
            else  // else just skip side info section if we have not already received and parsed detected obj list
            {
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Side Info i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            }
            
            sorterState = CHECK_TLV_TYPE;
            
            break;

        case READ_LOG_MAG_RANGE:
            {


              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_NOISE:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Noise Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
           
            break;
            
        case READ_AZIMUTH:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Azimuth Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_DOPPLER:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Doppler Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_STATS:
            {
        
              i = 0;
            
              while (i++ < tlvLen - 1)
              {
                     //ROS_INFO("DataUARTHandler Sort Thread : Parsing Stats Profile i=%d and tlvLen = %u", i, tlvLen);
              }
            
              currentDatap += tlvLen;
            
              sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_MRR_OBJ_STRUCT:
            {
                // Object type: mrr_18xx_dss -> dss_data_path.h: MmwDemo_detectedObjForTx
                
                // CHECK_TLV_TYPE code has already read tlvType and tlvLen

                i = 0;

                //get number of objects
                memcpy( &mmwData.numObjOut, &currentBufp->at(currentDatap), sizeof(mmwData.numObjOut));
                currentDatap += ( sizeof(mmwData.numObjOut) );
                
                // ROS_INFO("DataUARTHandler Sort Thread : case READ_MRR_OBJ_STRUCT, numObjOut = %u, subframe %d", mmwData.numObjOut, mmwData.header.subFrameNumber);
            
                //get xyzQFormat
                memcpy( &mmwData.xyzQFormat, &currentBufp->at(currentDatap), sizeof(mmwData.xyzQFormat));
                currentDatap += ( sizeof(mmwData.xyzQFormat) );
                
                // Detection (object) rviz message
                RScanObjectRviz->header.frame_id = frameID;
                RScanObjectRviz->height = 1;
                RScanObjectRviz->width = mmwData.numObjOut;
                RScanObjectRviz->is_dense = 1;
                RScanObjectRviz->points.resize(mmwData.numObjOut);
                
                // Detection (object) standard message
                RScanObjectStd->header.seq = 0;
                RScanObjectStd->header.stamp = ros::Time::now();
                RScanObjectStd->header.frame_id = frameID;
                RScanObjectStd->detections.resize(mmwData.numObjOut);

                // Populate pointcloud
                while( i < mmwData.numObjOut ) {
                    //get object range index (not used)
                    mmwData.objOut.rangeIdx = 0;

                    //get object doppler index. Relative velocity (m/s in oneQformat)
                    memcpy( &mmwData.objOut.dopplerIdx, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.dopplerIdx));
                    currentDatap += ( sizeof(mmwData.objOut.dopplerIdx) );
                    
                    //get object peak intensity value
                    memcpy( &mmwData.objOut.peakVal, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.peakVal));
                    currentDatap += ( sizeof(mmwData.objOut.peakVal) );
                    
                    //get object x-coordinate (meters in oneQformat)
                    memcpy( &mmwData.objOut.x, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.x));
                    currentDatap += ( sizeof(mmwData.objOut.x) );
                    
                    //get object y-coordinate (meters in oneQformat)
                    memcpy( &mmwData.objOut.y, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.y));
                    currentDatap += ( sizeof(mmwData.objOut.y) );
                    
                    //get object z-coordinate (meters in oneQformat)
                    memcpy( &mmwData.objOut.z, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.z));
                    currentDatap += ( sizeof(mmwData.objOut.z) );
                    
                    // Scan data processing
                    float temp[9];
                    float invXyzQFormat = 1.0 / (pow(2 , mmwData.xyzQFormat));
                    
                    // Position
                    temp[0] = (float) mmwData.objOut.x;
                    temp[1] = (float) mmwData.objOut.y;
                    temp[2] = (float) mmwData.objOut.z;
                    
                    // Speed
                    temp[3] = (float) mmwData.objOut.dopplerIdx;

                    // Convert from oneQformat to float
                    for (int j = 0; j < 4; j++) {
                        if (temp[j] > 32767) temp[j] -= 65536;
                        temp[j] = temp[j] * invXyzQFormat;
                    }

                    // Range
                    temp[4] = sqrt(temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2]);

                    // Intensity
                    temp[5] = 10 * log10(mmwData.objOut.peakVal + 1);

                    // Bearing
                    float angle = std::atan2(-temp[0], temp[1]);
                    temp[6] = angle / M_PI * 180;

                    // Range rate vetor
                    if (temp[3] >= 0.0)
                    {
                        temp[7] = temp[3] * std::cos(angle);
                        temp[8] = temp[3] * std::sin(angle);
                    }
                    else
                    {
                        angle += M_PI;
                        temp[7] = (-1.0 * temp[3]) * std::cos(angle);
                        temp[8] = (-1.0 * temp[3]) * std::sin(angle);
                    }

                    // Detection (object) rviz message
                    RScanObjectRviz->points[i].x = temp[1];   // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
                    RScanObjectRviz->points[i].y = -temp[0];  // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
                    RScanObjectRviz->points[i].z = temp[2];   // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis
                    RScanObjectRviz->points[i].intensity = temp[5];

                    // Detection (object) standard message
                    RScanObjectStd->detections[i].detection_id = i;
                    RScanObjectStd->detections[i].position.x = temp[1];
                    RScanObjectStd->detections[i].position.y = -temp[0];
                    RScanObjectStd->detections[i].position.z = 0.0;
                    RScanObjectStd->detections[i].velocity.x = temp[7];
                    RScanObjectStd->detections[i].velocity.y = temp[8];
                    RScanObjectStd->detections[i].velocity.z = 0.0;
                    RScanObjectStd->detections[i].amplitude = temp[5];

                    i++;
                }

                sorterState = CHECK_TLV_TYPE;
            }
            
            break;
            
        case READ_MRR_CLUSTER_STRUCT:
            {
                // Object type: mrr_18xx_dss -> dss_data_path.h: clusteringDBscanReportForTx

                // CHECK_TLV_TYPE code has already read tlvType and tlvLen

                i = 0;

                //get number of objects
                memcpy( &mmwData.numClusterOut, &currentBufp->at(currentDatap), sizeof(mmwData.numClusterOut));
                currentDatap += ( sizeof(mmwData.numClusterOut) );

                // ROS_INFO("DataUARTHandler Sort Thread : case READ_MRR_CLUSTER_STRUCT, numClusterOut = %u, subframe %d", mmwData.numClusterOut, mmwData.header.subFrameNumber);
            
                //get xyzQFormat
                memcpy( &mmwData.xyzQFormat, &currentBufp->at(currentDatap), sizeof(mmwData.xyzQFormat));
                currentDatap += ( sizeof(mmwData.xyzQFormat) );

                // Get time stamp
                ros::Time ts = ros::Time::now();
                std::string cluster_ns;

                if (mmwData.header.subFrameNumber == 0)
                {
                    cluster_ns = "cluster_0";
                }
                else
                {
                    cluster_ns = "cluster_1";
                }

                // Track (cluster) rviz message
                RScanClusterRviz->markers.resize(mmwData.numClusterOut);
                
                // Track (cluster) standard message
                RScanClusterStd->header.seq = 0;
                RScanClusterStd->header.stamp = ros::Time::now();
                RScanClusterStd->header.frame_id = frameID;
                RScanClusterStd->tracks.resize(mmwData.numClusterOut);
                
                // Populate pointcloud
                while( i < mmwData.numClusterOut ){
                    //get cluster x center
                    memcpy( &mmwData.clusterOut.xCenter, &currentBufp->at(currentDatap), sizeof(mmwData.clusterOut.xCenter));
                    currentDatap += ( sizeof(mmwData.clusterOut.xCenter) );

                    //get cluster y center
                    memcpy( &mmwData.clusterOut.yCenter, &currentBufp->at(currentDatap), sizeof(mmwData.clusterOut.yCenter));
                    currentDatap += ( sizeof(mmwData.clusterOut.yCenter) );

                    //get cluster x size
                    memcpy( &mmwData.clusterOut.xSize, &currentBufp->at(currentDatap), sizeof(mmwData.clusterOut.xSize));
                    currentDatap += ( sizeof(mmwData.clusterOut.xSize) );

                    //get cluster y size
                    memcpy( &mmwData.clusterOut.ySize, &currentBufp->at(currentDatap), sizeof(mmwData.clusterOut.ySize));
                    currentDatap += ( sizeof(mmwData.clusterOut.ySize) );
                    
                    // Scan data processing
                    float temp[4];
                    float invXyzQFormat = 1.0 / (pow(2 , mmwData.xyzQFormat));
                    
                    temp[0] = (float) mmwData.clusterOut.xCenter;
                    temp[1] = (float) mmwData.clusterOut.yCenter;
                    temp[2] = (float) mmwData.clusterOut.xSize;
                    temp[3] = (float) mmwData.clusterOut.ySize;

                    for (int j = 0; j < 4; j++) {
                        if (j < 2 && temp[j] > 32767) temp[j] -= 65536;
                        temp[j] = temp[j] * invXyzQFormat;
                    }

                    // Track (cluster) rviz message
                    RScanClusterRviz->markers[i].header.frame_id = frameID;
                    RScanClusterRviz->markers[i].header.stamp = ts;
                    RScanClusterRviz->markers[i].ns = cluster_ns;
                    RScanClusterRviz->markers[i].id = i;
                    RScanClusterRviz->markers[i].action = visualization_msgs::Marker::ADD;
                    RScanClusterRviz->markers[i].type = visualization_msgs::Marker::CUBE;
                    RScanClusterRviz->markers[i].lifetime = ros::Duration(1);
                    RScanClusterRviz->markers[i].pose.position.x = temp[1];
                    RScanClusterRviz->markers[i].pose.position.y = -temp[0];
                    RScanClusterRviz->markers[i].pose.position.z = 0.0;
                    RScanClusterRviz->markers[i].pose.orientation.x = 0.0;
                    RScanClusterRviz->markers[i].pose.orientation.y = 0.0;
                    RScanClusterRviz->markers[i].pose.orientation.z = 0.0;
                    RScanClusterRviz->markers[i].pose.orientation.w = 1.0;
                    RScanClusterRviz->markers[i].scale.x = temp[3];
                    RScanClusterRviz->markers[i].scale.y = temp[2];
                    RScanClusterRviz->markers[i].scale.z = 0.01;
                    RScanClusterRviz->markers[i].color.a = 1.0;
                    if (mmwData.header.subFrameNumber == 0)
                    {
                        RScanClusterRviz->markers[i].color.r = 0.0;
                        RScanClusterRviz->markers[i].color.g = 0.0;
                        RScanClusterRviz->markers[i].color.b = 1.0;
                    }
                    else
                    {
                        RScanClusterRviz->markers[i].color.r = 0.0;
                        RScanClusterRviz->markers[i].color.g = 0.0;
                        RScanClusterRviz->markers[i].color.b = 0.5;
                    }

                    // Track (cluster) standard message
                    RScanClusterStd->tracks[i].track_id = i;
                    RScanClusterStd->tracks[i].track_shape.points.resize(4);
                    RScanClusterStd->tracks[i].track_shape.points[0].x = temp[1] - temp[3] / 2;
                    RScanClusterStd->tracks[i].track_shape.points[0].y = -temp[0] - temp[2] / 2;
                    RScanClusterStd->tracks[i].track_shape.points[0].z = 0.0;
                    RScanClusterStd->tracks[i].track_shape.points[1].x = temp[1] - temp[3] / 2;
                    RScanClusterStd->tracks[i].track_shape.points[1].y = -temp[0] + temp[2] / 2;
                    RScanClusterStd->tracks[i].track_shape.points[1].z = 0.0;
                    RScanClusterStd->tracks[i].track_shape.points[2].x = temp[1] + temp[3] / 2;
                    RScanClusterStd->tracks[i].track_shape.points[2].y = -temp[0] + temp[2] / 2;
                    RScanClusterStd->tracks[i].track_shape.points[2].z = 0.0;
                    RScanClusterStd->tracks[i].track_shape.points[3].x = temp[1] + temp[3] / 2;
                    RScanClusterStd->tracks[i].track_shape.points[3].y = -temp[0] - temp[2] / 2;
                    RScanClusterStd->tracks[i].track_shape.points[3].z = 0.0;
                    RScanClusterStd->tracks[i].linear_velocity.x = 0.0;
                    RScanClusterStd->tracks[i].linear_velocity.y = 0.0;
                    RScanClusterStd->tracks[i].linear_velocity.z = 0.0;

                    i++;
                }

                sorterState = CHECK_TLV_TYPE;
            }

            break;
            
        case READ_MRR_TRACKED_OBJ_STRUCT:
            {
                // Object type: mrr_18xx_dss -> dss_data_path.h: trackingReportForTx_t

                // CHECK_TLV_TYPE code has already read tlvType and tlvLen

                i = 0;

                //get number of objects
                memcpy( &mmwData.numTrackOut, &currentBufp->at(currentDatap), sizeof(mmwData.numTrackOut));
                currentDatap += ( sizeof(mmwData.numTrackOut) );

                // ROS_INFO("DataUARTHandler Sort Thread : case READ_MRR_TRACKED_OBJ_STRUCT, numTrackOut = %u, subframe %d", mmwData.numTrackOut, mmwData.header.subFrameNumber);
            
                //get xyzQFormat
                memcpy( &mmwData.xyzQFormat, &currentBufp->at(currentDatap), sizeof(mmwData.xyzQFormat));
                currentDatap += ( sizeof(mmwData.xyzQFormat) );

                // Get time stamp
                ros::Time ts = ros::Time::now();
                std::string track_ns;

                if (mmwData.header.subFrameNumber == 0)
                {
                    track_ns = "track_0";
                }
                else
                {
                    track_ns = "track_1";
                }

                // Track (tracked object) rviz message
                RScanTrackRviz->markers.resize(mmwData.numTrackOut);
                
                // Track (tracked object) standard message
                RScanTrackStd->header.seq = 0;
                RScanTrackStd->header.stamp = ros::Time::now();
                RScanTrackStd->header.frame_id = frameID;
                RScanTrackStd->tracks.resize(mmwData.numTrackOut);
                
                // Populate pointcloud
                while( i < mmwData.numTrackOut ){
                    //get track x center
                    memcpy( &mmwData.trackOut.x, &currentBufp->at(currentDatap), sizeof(mmwData.trackOut.x));
                    currentDatap += ( sizeof(mmwData.trackOut.x) );

                    //get track y center
                    memcpy( &mmwData.trackOut.y, &currentBufp->at(currentDatap), sizeof(mmwData.trackOut.y));
                    currentDatap += ( sizeof(mmwData.trackOut.y) );

                    //get track x velocity
                    memcpy( &mmwData.trackOut.xVel, &currentBufp->at(currentDatap), sizeof(mmwData.trackOut.xVel));
                    currentDatap += ( sizeof(mmwData.trackOut.xVel) );

                    //get track y velocity
                    memcpy( &mmwData.trackOut.yVel, &currentBufp->at(currentDatap), sizeof(mmwData.trackOut.yVel));
                    currentDatap += ( sizeof(mmwData.trackOut.yVel) );

                    //get track x size
                    memcpy( &mmwData.trackOut.xSize, &currentBufp->at(currentDatap), sizeof(mmwData.trackOut.xSize));
                    currentDatap += ( sizeof(mmwData.trackOut.xSize) );

                    //get track y size
                    memcpy( &mmwData.trackOut.ySize, &currentBufp->at(currentDatap), sizeof(mmwData.trackOut.ySize));
                    currentDatap += ( sizeof(mmwData.trackOut.ySize) );
                    
                    // Scan data processing
                    float temp[6];
                    float invXyzQFormat = 1.0 / (pow(2 , mmwData.xyzQFormat));
                    
                    temp[0] = (float) mmwData.trackOut.x;
                    temp[1] = (float) mmwData.trackOut.y;
                    temp[2] = (float) mmwData.trackOut.xVel;
                    temp[3] = (float) mmwData.trackOut.yVel;
                    temp[4] = (float) mmwData.trackOut.xSize;
                    temp[5] = (float) mmwData.trackOut.ySize;

                    for (int j = 0; j < 6; j++) {
                        if (j < 4 && temp[j] > 32767) temp[j] -= 65536;
                        temp[j] = temp[j] * invXyzQFormat;
                    }

                    // Track (tracked object) rviz message
                    RScanTrackRviz->markers[i].header.frame_id = frameID;
                    RScanTrackRviz->markers[i].header.stamp = ts;
                    RScanTrackRviz->markers[i].ns = track_ns;
                    RScanTrackRviz->markers[i].id = i;
                    RScanTrackRviz->markers[i].action = visualization_msgs::Marker::ADD;
                    RScanTrackRviz->markers[i].type = visualization_msgs::Marker::CUBE;
                    RScanTrackRviz->markers[i].lifetime = ros::Duration(1);
                    RScanTrackRviz->markers[i].pose.position.x = temp[1];
                    RScanTrackRviz->markers[i].pose.position.y = -temp[0];
                    RScanTrackRviz->markers[i].pose.position.z = 0.0;
                    RScanTrackRviz->markers[i].pose.orientation.x = 0.0;
                    RScanTrackRviz->markers[i].pose.orientation.y = 0.0;
                    RScanTrackRviz->markers[i].pose.orientation.z = 0.0;
                    RScanTrackRviz->markers[i].pose.orientation.w = 1.0;
                    RScanTrackRviz->markers[i].scale.x = temp[5];
                    RScanTrackRviz->markers[i].scale.y = temp[4];
                    RScanTrackRviz->markers[i].scale.z = 0.01;
                    RScanTrackRviz->markers[i].color.a = 1.0;
                    if (mmwData.header.subFrameNumber == 0)
                    {
                        RScanTrackRviz->markers[i].color.r = 0.0;
                        RScanTrackRviz->markers[i].color.g = 1.0;
                        RScanTrackRviz->markers[i].color.b = 0.0;
                    }
                    else
                    {
                        RScanTrackRviz->markers[i].color.r = 0.0;
                        RScanTrackRviz->markers[i].color.g = 0.5;
                        RScanTrackRviz->markers[i].color.b = 0.0;
                    }

                    // Track (tracked object) standard message
                    RScanTrackStd->tracks[i].track_id = i;
                    RScanTrackStd->tracks[i].track_shape.points.resize(4);
                    RScanTrackStd->tracks[i].track_shape.points[0].x = temp[1] - temp[5] / 2;
                    RScanTrackStd->tracks[i].track_shape.points[0].y = -temp[0] - temp[4] / 2;
                    RScanTrackStd->tracks[i].track_shape.points[0].z = 0.0;
                    RScanTrackStd->tracks[i].track_shape.points[1].x = temp[1] - temp[5] / 2;
                    RScanTrackStd->tracks[i].track_shape.points[1].y = -temp[0] + temp[4] / 2;
                    RScanTrackStd->tracks[i].track_shape.points[1].z = 0.0;
                    RScanTrackStd->tracks[i].track_shape.points[2].x = temp[1] + temp[5] / 2;
                    RScanTrackStd->tracks[i].track_shape.points[2].y = -temp[0] + temp[4] / 2;
                    RScanTrackStd->tracks[i].track_shape.points[2].z = 0.0;
                    RScanTrackStd->tracks[i].track_shape.points[3].x = temp[1] + temp[5] / 2;
                    RScanTrackStd->tracks[i].track_shape.points[3].y = -temp[0] - temp[4] / 2;
                    RScanTrackStd->tracks[i].track_shape.points[3].z = 0.0;
                    RScanTrackStd->tracks[i].linear_velocity.x = temp[3];
                    RScanTrackStd->tracks[i].linear_velocity.y = temp[2];
                    RScanTrackStd->tracks[i].linear_velocity.z = 0.0;

                    i++;
                }

                sorterState = CHECK_TLV_TYPE;
            }

            break;

            break;
            
        case READ_MRR_PARKING_STRUCT:
            {
                // ROS_INFO("DataUARTHandler Sort Thread : case READ_MRR_PARKING_STRUCT, tlvLen = %u, subframe %d", tlvLen, mmwData.header.subFrameNumber);
                
                i = 0;
                
                while (i++ < tlvLen - 1)
                {
                    // ROS_INFO("DataUARTHandler Sort Thread : Parsing Parking i=%d, tlvLen = %u, subframe %d", i, tlvLen, mmwData.header.subFrameNumber);
                }
                
                currentDatap += tlvLen;
                
                sorterState = CHECK_TLV_TYPE;
            }

            break;
        
        case CHECK_TLV_TYPE:
        
            // ROS_INFO("DataUARTHandler Sort Thread : case CHECK_TLV_TYPE, tlvCount = %d, numTLV = %d", tlvCount, mmwData.header.numTLVs);
        
            if(tlvCount++ >= mmwData.header.numTLVs)  // Done parsing all received TLV sections
            {
                if (radarBin == BIN_XWR18XX_MRR)
                {
                    // Publish detected object pointcloud
                    if (mmwData.numObjOut > 0)
                    {
                        if (mmwData.header.subFrameNumber == 1)
                        {
                            DataUARTHandler_pub_1.publish(RScanObjectRviz);
                            radar_scan_pub_1.publish(RScanObjectStd);
                        }
                        else
                        {
                            DataUARTHandler_pub.publish(RScanObjectRviz);
                            radar_scan_pub.publish(RScanObjectStd);
                        }
                    }

                    // Publish detected cluster marker array
                    if (mmwData.numClusterOut > 0)
                    {
                        if (mmwData.header.subFrameNumber == 1)
                        {
                            DataUARTHandler_pub_3.publish(RScanClusterRviz);
                            radar_scan_pub_3.publish(RScanClusterStd);
                        }
                        else
                        {
                            // MRR clusters ignored
                        }
                    }

                    // Publish detected track marker array
                    if (mmwData.numTrackOut > 0)
                    {
                        if (mmwData.header.subFrameNumber == 1)
                        {
                            // USRR tracked objects ignored
                        }
                        else
                        {
                            DataUARTHandler_pub_2.publish(RScanTrackRviz);
                            radar_scan_pub_2.publish(RScanTrackStd);
                        }
                    }
                }
                else
                {
                    // Publish detected object pointcloud
                    if (mmwData.numObjOut > 0)
                    {
                        j = 0;
                        for (i = 0; i < mmwData.numObjOut; i++)
                        {
                            // Keep point if elevation and azimuth angles are less than specified max values
                            // (NOTE: The following calculations are done using ROS standard coordinate system axis definitions where X is forward and Y is left)
                            if (((maxElevationAngleRatioSquared == -1) ||
                                (((RScan->points[i].z * RScan->points[i].z) / (RScan->points[i].x * RScan->points[i].x +
                                                                                RScan->points[i].y * RScan->points[i].y)
                                ) < maxElevationAngleRatioSquared)
                                ) &&
                                ((maxAzimuthAngleRatio == -1) || (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) &&
                                        (RScan->points[i].x != 0)
                            )
                            {
                                //ROS_INFO("Kept point");
                                // copy: points[i] => points[j]
                                memcpy( &RScan->points[j], &RScan->points[i], sizeof(RScan->points[i]));
                                j++;
                            }
                        }
                        mmwData.numObjOut = j;  // update number of objects as some points may have been removed

                        // Resize point cloud since some points may have been removed
                        RScan->width = mmwData.numObjOut;
                        RScan->points.resize(RScan->width * RScan->height);
                        
                        //ROS_INFO("mmwData.numObjOut after = %d", mmwData.numObjOut);
                        //ROS_INFO("DataUARTHandler Sort Thread: number of obj = %d", mmwData.numObjOut );
                        
                        DataUARTHandler_pub.publish(RScan);
                    }
                }

                //ROS_INFO("DataUARTHandler Sort Thread : CHECK_TLV_TYPE state says tlvCount max was reached, going to switch buffer state");
                sorterState = SWAP_BUFFERS;
            }
            
            else  // More TLV sections to parse
            {
               //get tlvType (32 bits) & remove from queue
                memcpy( &tlvType, &currentBufp->at(currentDatap), sizeof(tlvType));
                currentDatap += ( sizeof(tlvType) );
                
                //ROS_INFO("DataUARTHandler Sort Thread : sizeof(tlvType) = %d", sizeof(tlvType));
            
                //get tlvLen (32 bits) & remove from queue
                memcpy( &tlvLen, &currentBufp->at(currentDatap), sizeof(tlvLen));
                currentDatap += ( sizeof(tlvLen) );
                
                //ROS_INFO("DataUARTHandler Sort Thread : sizeof(tlvLen) = %d", sizeof(tlvLen));
                
                //ROS_INFO("DataUARTHandler Sort Thread : tlvType = %d, tlvLen = %d", (int) tlvType, tlvLen);
            
                if (radarBin == BIN_XWR18XX_MRR)
                {
                    switch(tlvType)
                    {
                    case MMWDEMO_OUTPUT_MSG_NULL:
                    
                        break;
                    
                    case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:
                        sorterState = READ_MRR_OBJ_STRUCT;
                        break;
                    
                    case MMWDEMO_OUTPUT_MSG_CLUSTERS:
                        sorterState = READ_MRR_CLUSTER_STRUCT;
                        break;
                    
                    case MMWDEMO_OUTPUT_MSG_TRACKED_OBJECTS:
                        sorterState = READ_MRR_TRACKED_OBJ_STRUCT;
                        break;
                    
                    case MMWDEMO_OUTPUT_MSG_PARKING_ASSIST:
                        sorterState = READ_MRR_PARKING_STRUCT;
                        break;
                    
                    default: break;
                    }
                }
                else
                {
                    switch(tlvType)
                    {
                    case MMWDEMO_OUTPUT_MSG_NULL:
                    
                        break;
                    
                    case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:
                        //ROS_INFO("DataUARTHandler Sort Thread : Object TLV");
                        sorterState = READ_OBJ_STRUCT;
                        break;
                    
                    case MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:
                        //ROS_INFO("DataUARTHandler Sort Thread : Range TLV");
                        sorterState = READ_LOG_MAG_RANGE;
                        break;
                    
                    case MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:
                        //ROS_INFO("DataUARTHandler Sort Thread : Noise TLV");
                        sorterState = READ_NOISE;
                        break;
                    
                    case MMWDEMO_OUTPUT_MSG_AZIMUTH_STATIC_HEAT_MAP:
                        //ROS_INFO("DataUARTHandler Sort Thread : Azimuth Heat TLV");
                        sorterState = READ_AZIMUTH;
                        break;
                    
                    case MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
                        //ROS_INFO("DataUARTHandler Sort Thread : R/D Heat TLV");
                        sorterState = READ_DOPPLER;
                        break;
                    
                    case MMWDEMO_OUTPUT_MSG_STATS:
                        //ROS_INFO("DataUARTHandler Sort Thread : Stats TLV");
                        sorterState = READ_STATS;
                        break;
                    
                    case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO:
                        //ROS_INFO("DataUARTHandler Sort Thread : Side info TLV");
                        sorterState = READ_SIDE_INFO;
                        break;

                    case MMWDEMO_OUTPUT_MSG_MAX:
                        //ROS_INFO("DataUARTHandler Sort Thread : Header TLV");
                        sorterState = READ_HEADER;
                        break;
                    
                    default: break;
                    }
                }
            }
            
        break;
            
       case SWAP_BUFFERS:
       
            pthread_mutex_lock(&countSync_mutex);
            pthread_mutex_unlock(&currentBufp_mutex);
                            
            countSync++;
                
            if(countSync == COUNT_SYNC_MAX)
            {
                pthread_cond_signal(&countSync_max_cv);
            }
                
            pthread_cond_wait(&sort_go_cv, &countSync_mutex);
                
            pthread_mutex_unlock(&countSync_mutex);
            pthread_mutex_lock(&currentBufp_mutex);
                
            currentDatap = 0;
            tlvCount = 0;
                
            sorterState = READ_HEADER;
            
            break;
                
            
        default: break;
        }
    }
    
    
    pthread_exit(NULL);
}

void DataUARTHandler::start(void)
{
    
    pthread_t uartThread, sorterThread, swapThread;
    
    int  iret1, iret2, iret3;
    
    pthread_mutex_init(&countSync_mutex, NULL);
    pthread_mutex_init(&nextBufp_mutex, NULL);
    pthread_mutex_init(&currentBufp_mutex, NULL);
    pthread_cond_init(&countSync_max_cv, NULL);
    pthread_cond_init(&read_go_cv, NULL);
    pthread_cond_init(&sort_go_cv, NULL);
    
    countSync = 0;
    
    /* Create independent threads each of which will execute function */
    iret1 = pthread_create( &uartThread, NULL, this->readIncomingData_helper, this);
    if(iret1)
    {
     ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
     ros::shutdown();
    }
    
    iret2 = pthread_create( &sorterThread, NULL, this->sortIncomingData_helper, this);
    if(iret2)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }
    
    iret3 = pthread_create( &swapThread, NULL, this->syncedBufferSwap_helper, this);
    if(iret3)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }
    
    ros::spin();

    pthread_join(iret1, NULL);
    ROS_INFO("DataUARTHandler Read Thread joined");
    pthread_join(iret2, NULL);
    ROS_INFO("DataUARTHandler Sort Thread joined");
    pthread_join(iret3, NULL);
    ROS_INFO("DataUARTHandler Swap Thread joined");
    
    pthread_mutex_destroy(&countSync_mutex);
    pthread_mutex_destroy(&nextBufp_mutex);
    pthread_mutex_destroy(&currentBufp_mutex);
    pthread_cond_destroy(&countSync_max_cv);
    pthread_cond_destroy(&read_go_cv);
    pthread_cond_destroy(&sort_go_cv);
    
    
}

void* DataUARTHandler::readIncomingData_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->readIncomingData());
}

void* DataUARTHandler::sortIncomingData_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->sortIncomingData());
}

void* DataUARTHandler::syncedBufferSwap_helper(void *context)
{  
    return (static_cast<DataUARTHandler*>(context)->syncedBufferSwap());
}

void DataUARTHandler::visualize(const ti_mmwave_rospkg::RadarScan &msg){
    visualization_msgs::Marker marker;

    marker.header.frame_id = frameID;
    marker.header.stamp = ros::Time::now();
    marker.id = msg.point_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration(tfr);
    marker.action = marker.ADD;

    marker.pose.position.x = msg.x;
    marker.pose.position.y = msg.y;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 0;

    marker.scale.x = .03;
    marker.scale.y = .03;
    marker.scale.z = .03;
    
    marker.color.a = 1;
    marker.color.r = (int) 255 * msg.intensity;
    marker.color.g = (int) 255 * msg.intensity;
    marker.color.b = 1;

    marker_pub.publish(marker);
}