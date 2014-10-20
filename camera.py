'''
Created on Jun 12, 2013

@author: enalepa
'''
import Image, qha_tools, time

class Camera(object):
    def __init__(self, videoDev ,name, cameraIndexName, resolutionName, colorSpaceName, fps):

        cameraIndexesName = ["TopCamera", "BottomCamera"]
        resolutionsName = ["160*120", "320*240", "640*480", "1280*960", "", "", "", "80*60"]
        colorSpacesName = ["Y", "U", "V", "R", "G", "B", "H", "S", "Y", "0xY'Y'VVYYUU", "0xVVUUYY", "0xBBGGRR", "0xYYSSHH", "0xRRGGBB", "TIFF", "HSY", "HS"]
        parametersName =    [
                            "Brightness", "Contrast", "Saturation", "Hue", "", "", "Gain", "Horizontal Flip", "Vertical Flip", "", "", "Auto Exposition", "Auto White Balance", "",
                            "Camera Resolution", "Frames Per Second", "", "Exposure", "Camera Select", "Reset camera registers", "", "", "Auto Exposure Algorithm", "", "Sharpness",
                            "", "", "", "", "", "", "", "", "White Balance", "Back light compensation"
                        ]

        self.videoDevice=videoDev
        cameraIndex = cameraIndexesName.index(cameraIndexName)
        resolution = resolutionsName.index(resolutionName)
        colorSpace = colorSpacesName.index(colorSpaceName)

        cameraAlreadySubscribed = False
        for subscriber in videoDev.getSubscribers():
            if subscriber == name + "_0":
                cameraAlreadySubscribed = True
                self.name = subscriber

        if not cameraAlreadySubscribed:
            self.name = videoDev.subscribeCamera(name, cameraIndex, resolution, colorSpace, fps)
            self.name = videoDev.unsubscribe(self.name)
            self.name = videoDev.subscribeCamera(name, cameraIndex, resolution, colorSpace, fps)

        autoExposition = int(qha_tools.readParameter("Configuration/parameters.cfg","Camera","AutoExposition"))
        saturation = int(qha_tools.readParameter("Configuration/parameters.cfg","Camera","Saturation"))
        gain = int(qha_tools.readParameter("Configuration/parameters.cfg","Camera","Gain"))
        exposure = int(qha_tools.readParameter("Configuration/parameters.cfg","Camera","Exposure"))

        self.videoDevice.setParameter(cameraIndex, parametersName.index("Auto Exposition"),autoExposition)
        self.videoDevice.setParameter(cameraIndex, parametersName.index("Saturation"),saturation )
        self.videoDevice.setParameter(cameraIndex, parametersName.index("Gain"),gain )
        self.videoDevice.setParameter(cameraIndex, parametersName.index("Exposure"),exposure )

        self.images = []

    def ImageTake(self,cropBox=()):
        imageRaw = self.videoDevice.getImageRemote(self.name)
        tmpIm = ImageContainer(imageRaw)
        self.images.append(tmpIm)
        if cropBox == ():
            self.images.append(tmpIm.imagePIL)
        else:
            self.images.append(tmpIm.imagePIL.crop(cropBox))


    def ImageRAZ(self):
        self.images = []

class ImageContainer(object):
    def __init__(self, imageRaw):
        self.width = imageRaw[0]
        self.height = imageRaw[1]
        self.layersNumber = imageRaw[2]
        self.colorSpace = imageRaw[3]
        self.timeStamp = imageRaw[4], imageRaw[5]
        self.image = imageRaw[6]
        self.cameraID = imageRaw[7]
        self.leftAngle = imageRaw[8]
        self.topAngle = imageRaw[9]
        self.rightAngle = imageRaw[10]
        self.bottomAngle = imageRaw[11]

        for i,codeASCCI in enumerate(self.image):
            self.image[i]=chr(codeASCCI)
        self.image=str(self.image)

        self.imagePIL = Image.fromstring("RGB", (self.width, self.height), self.image)


    def show(self):
        self.imagePIL.show()

    def save(self, path):
        self.imagePIL.save(path)


