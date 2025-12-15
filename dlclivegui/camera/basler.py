"""
DeepLabCut Toolbox (deeplabcut.org)
© A. & M. Mathis Labs

Licensed under GNU Lesser General Public License v3.0
"""

#import pypylon as pylon
from pypylon import pylon
from imutils import rotate_bound
import time

from dlclivegui.camera import Camera, CameraError
TIMEOUT = 50 # 1000



def get_devices():
    tlFactory = pylon.TlFactory.GetInstance()
    devices = tlFactory.EnumerateDevices()
    return devices

class BaslerCam(Camera):
    @staticmethod
    def arg_restrictions():
        """ Returns a dictionary of arguments restrictions for DLCLiveGUI
        """
        devices = get_devices()
        device_ids = list(range(len(devices)))
        return {"device": device_ids, "display": [True, False]}

    def __init__(
        self,
        device=0,
        resolution=[640, 480],
        exposure=15000,
        rotate=0,
        crop=None,
        gain=0.0,
        fps=30,
        display=True,
        display_resize=1.0,
    ):

        super().__init__(
            device,
            resolution=resolution,
            exposure=exposure,
            rotate=rotate,
            crop=crop,
            gain=gain,
            fps=fps,
            use_tk_display=display,
            display_resize=display_resize,
        )

        self.display = display

    def set_capture_device(self):

        devices = get_devices()
        self.cam = pylon.InstantCamera(
            pylon.TlFactory.GetInstance().CreateDevice(devices[self.id])
        )
        self.cam.Open()

        self.cam.Gain.SetValue(self.gain)
        self.cam.ExposureTime.SetValue(self.exposure)
        self.cam.Width.SetValue(self.im_size[0])
        self.cam.Height.SetValue(self.im_size[1])

        # the following line is commented out by JIAAO.
        # For better synchronization, we want to avoid starting acquisition here,
        # and use a trigger later on.
        # self.cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        # JIAAO ADDED
        # for k in dir(self.cam):
            # print(k)
        # input("......")
        # print(dir(self.cam.UserSetSelector))
        # print("====")
        # print(self.cam.UserSetSelector.__doc__)
        #str_val = self.cam.UserSetSelector.FromString("UserSet1").ToString()
        #str_val = "UserSet1"
        #self.cam.UserSetSelector.SetValue(str_val) # pylon.UserSetSelector_UserSet1);
        # Usersets are defined in the Pylon GUI software and stored in camera.
        # We use it for digital IO signal configurations
        self.cam.UserSetSelector = "UserSet1"
        print("Basler - Loading current UserSet:", self.cam.UserSetSelector.Value)
        self.cam.UserSetLoad.Execute()
        self.cam.AcquisitionFrameRate = int(1.2*self.fps) # slightly more than 80 FPS which is software polling rate
        self.cam.Width.SetValue(self.im_size[0])
        self.cam.Height.SetValue(self.im_size[1])
        # END JIAAO ADDED

        return True
    
    def start_acquisition(self):
        self.cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    def get_image(self):
        # JIAAO EDITED
        frame = None
        try:
            grabResult = self.cam.RetrieveResult(
                TIMEOUT, pylon.TimeoutHandling_ThrowException)
            # print(dir(grabResult))
            if not grabResult.IsValid():
                return frame
            if grabResult.GrabSucceeded():
                image = self.converter.Convert(grabResult)
                frame = image.GetArray()
                if self.rotate:
                    frame = rotate_bound(frame, self.rotate)
                if self.crop:
                    frame = frame[self.crop[2]: self.crop[3],
                                  self.crop[0]: self.crop[1]]
            else:
                #JIAAO ADDED
                pass
                # raise CameraError("Basler Camera did not return an image!") # commented out by JIAAO
                # END JIAAO ADDED
            grabResult.Release()
            
        except Exception as e:
            print("basler.get_image() timed out")
            print("Exception grabbing result:", type(e), "===========\n", e)
            pass

        return frame

    def close_capture_device(self):

        self.cam.StopGrabbing()