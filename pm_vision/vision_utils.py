import cv2 # OpenCV library

def match_vision_function(frame_processed, FileData, key):
    match key:
        case "theshold":
            function_parameter = FileData [key]
            active = function_parameter['active']
            thresh = function_parameter['thresh']
            maxval = function_parameter['maxval']
            type = function_parameter['type'] 
            if active == 'True':
                _Command = "cv2." + type
                _,frame_processed = cv2.threshold(frame_processed,thresh,maxval,exec(_Command))
                print("Theshold executed")
        case "adaptiveThreshold":
            function_parameter = FileData [key]
            active = function_parameter['active']
            maxValue = function_parameter['maxValue']
            adaptiveMethod = function_parameter['adaptiveMethod']
            thresholdType = function_parameter['thresholdType']
            blockSize = function_parameter['blockSize'] 
            C_Value = function_parameter['C']
            if active == 'True':
                _Command_adaptiveMethod = "cv2." + adaptiveMethod
                _Command_thresholdType = "cv2." + thresholdType
                frame_processed = cv2.adaptiveThreshold(frame_processed,maxValue,exec(_Command_adaptiveMethod),exec(_Command_thresholdType),blockSize,C_Value)
                print("Adaptive Theshold executed")
        case "BGR2GRAY":
            function_parameter = FileData [key]
            active = function_parameter['active']
            if active == 'True':
                frame_processed = cv2.cvtColor(frame_processed, cv2.COLOR_BGR2GRAY)
                print("BGR2GRAY executed")
        case "Canny":
            function_parameter = FileData [key]
            active = function_parameter['active']
            threshold1 = function_parameter['threshold1']
            threshold2 = function_parameter['threshold2']
            aperatureSize = function_parameter['aperatureSize']
            L2gradient = function_parameter['L2gradient'] 
            if active == 'True':
                frame_processed = edges = cv2.Canny(frame_processed,threshold1,threshold2,aperatureSize)
                print("Canny executed")
    return frame_processed