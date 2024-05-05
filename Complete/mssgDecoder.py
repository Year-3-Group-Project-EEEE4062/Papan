import struct

class decoderMssg:
    def __decodeData(self, mssg):
        # data type identifier
        integer_identifier = 0x01
        float_identifier = 0x02

        # Essentials for decoding message
        mssgStartingIndex = 6
        intBufferSize = 4
        floatBufferSize = 4
        
        # Pick out where the data type identifier is stored in the message
        dataType_identifier = mssg[1]
        
        # Get the length of the information
        dataLength = struct.unpack('i', mssg[2:mssgStartingIndex])[0]

        # Decode data based on the stored data type identifier
        if dataType_identifier == float_identifier:
            float_value = struct.unpack('f'* dataLength, mssg[mssgStartingIndex:mssgStartingIndex+(floatBufferSize*dataLength)])
            float_value = [round(num,6) for num in float_value]
            print(float_value)
            return float_value # return decoded message

        elif dataType_identifier == integer_identifier:
            integer_value = struct.unpack('i'* dataLength, mssg[mssgStartingIndex:mssgStartingIndex+(intBufferSize*dataLength)])
            print(integer_value)
            return integer_value # return decoded message

        else:
            raise ValueError("Unknown identifier")
               
    def getNewMssg(self, mssg):
        """
        Function to get the decoded version of the new message received from pico .

        Returns:
            char: character representing whether remote or auto message
            instruction: containing information about the decoded message
        """
        # mode identifier
        remote_identifier = 0x01
        auto_identifier = 0x02
        temp_identifier = 0x04
        print("Processing message...")

        # Extract the mode identifier
        mode_identifier = mssg[0]

        # Check for valid message length
        if(len(mssg)>3):
            # Check for valid mode
            if mode_identifier == remote_identifier:
                # User chooses remote mode
                instruction = self.__decodeData(mssg)
                return 'r', instruction

            elif mode_identifier == auto_identifier:
                # User chooses auto mode
                instruction = self.__decodeData(mssg)
                return 'a', instruction
            elif mode_identifier == temp_identifier:
                instruction = self.__decodeData(mssg)
                return instruction                
            else:
                # Invalid mode
                print("Invalid mode!")
                return None, None
                


