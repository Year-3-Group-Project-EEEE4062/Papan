import struct

class decoderMssg:
    def __init__(self):
        # As a way to keep track if there is new messages to be processed or not
        self.isThereNewMssg = False
        
    def resetFlag(self):
        self.isThereNewMssg = False
        
    def checkIfNewMssg(self):
        return self.isThereNewMssg
    
    def setNewMssg(self, mssg):
        # Set the new message locally in class
        self.newMssg = mssg

        # Change bool to True to let know there is new messages
        self.isThereNewMssg = True

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
               
    def getNewMssg(self):
        """
        Function to get the decoded version of the new message received from pico .

        Returns:
            char: character representing whether remote or auto message
            instruction: containing information about the decoded message
        """
        # mode identifier
        remote_identifier = 0x01
        auto_identifier = 0x02
        print("Processing message...")

        # Extract the mode identifier
        mode_identifier = self.newMssg[0]

        # Check for valid message length
        if(len(self.newMssg)!=1):
            # Check for valid mode
            if mode_identifier == remote_identifier:
                # User chooses remote mode
                instruction = self.__decodeData(self.newMssg)
                return 'r', instruction

            elif mode_identifier == auto_identifier:
                # User chooses auto mode
                instruction = self.__decodeData(self.newMssg)
                return 'a', instruction

            else:
                # Invalid mode
                print("Invalid mode!")
                return None, None
                