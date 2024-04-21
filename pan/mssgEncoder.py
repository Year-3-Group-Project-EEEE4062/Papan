import struct

class encoderMssg:
    def __init__(self):
        # mode identifier
        self.remote_identifier = 0x01
        self.auto_identifier = 0x02
        
    def packFloats(self , mode, mssg):
        """
        Function to pack message of type double to list of uint8.

        Args:
            mode: either remote (int 0) or auto (int 1)
            mssgType: to let user know what kind of message is it (any int)
            mssg: list of float

        Returns:
            encoded: unsigned 8-bit integer list (mssg back to Medium).
        """
        float_identifier = 0x02
        encoded = bytearray()

        if(mode==0):
            # message has something to do with remote mode
            encoded.extend(self.remote_identifier.to_bytes(1,'big')) # data type identifier
        elif(mode==1):
            # message has something to do with auto mode'
            encoded.extend(self.auto_identifier.to_bytes(1,'big')) # data type identifier

        encoded.extend(float_identifier.to_bytes(1,'little')) # data type identifier
        encoded.extend(struct.pack('i', len(mssg))) # how many data to extracted
        encoded.extend(struct.pack('f' * len(mssg), *mssg)) # the data itself

        return encoded
        
    def packIntegers(self , mode, mssg):
        """
        Function to pack message of type int to list of uint8.

        Args:
            mode: either remote (int 0) or auto (int 1)
            mssgType: to let user know what kind of message is it (any int)
            mssg: list of int

        Returns:
            encoded: unsigned 8-bit integer list (mssg back to Medium).
        """

        integer_identifier = 0x01
        encoded = bytearray()

        if(mode==0):
            # message has something to do with remote mode
            encoded.extend(self.remote_identifier.to_bytes(1,'big')) # data type identifier
        elif(mode==1):
            # message has something to do with auto mode'
            encoded.extend(self.auto_identifier.to_bytes(1,'big')) # data type identifier

        encoded.extend(integer_identifier.to_bytes(1,'little')) # data type identifier
        encoded.extend(struct.pack('i', len(mssg))) # how many data to extracted
        encoded.extend(struct.pack('i' * len(mssg), *mssg)) # the data itself

        return encoded