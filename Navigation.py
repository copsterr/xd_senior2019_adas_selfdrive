class Navigation(object):
    def __init__(self, actor):
        t = actor.get_transform()
        self.x = t.location.x
        self.y = t.location.y

        # each dictionary has different condition to check area
        self.path_longY = {"path1_1": (1.9, 156), "path1_2": (1.3, 39.2), "path1_3": (5.5, -110.8), "path2_2": (239.7, 98.4),
                           "path2_3": (241, 27), "path2_5": (152.2, -44.4), "path4": (-145.8, 84.2), "path5": (-145.8,16.8)
                           , "path8": (-78, 37), "path9": (-88.9, -44.4), "path11": (232.7, -48.6), "path15": (-84, -175)
                           , "path17": (-78.1, -104), "path19": (-3.5, -174.5), "path21": (7, -170.6), "path24": (79, -171)
                           , "path25": (84.1, -113), "path26": (77.1, -108.8), "path29": (82.1, -57.1), "path30": (74.5, -39.8)
                           , "path31": (169.5, 13)}
        self.direction_longY = {"path1_1": "Go Straight", "path1_2": "Go Straight", "path1_3": "Turn Right",
                                "path2_2": "Go Straight", "path2_3": "Turn Left", "path2_5": "Go Straight", "path4": "Go Straight",
                                "path5": "Turn Right", "path8": "Turn Right", "path9": "Turn Left", "path11": "Turn Right"
                                , "path15": "Turn Left", "path17": "Turn Right", "path19": "Turn Left", "path21":"Turn Right"
                                , "path24": "Turn Left", "path25": "Turn Right", "path26": "Turn Left", "path29": "Turn Right",
                                "path30": "Turn Right", "path31": "Turn Right"}

        self.path_longX = {"path1_4": (48.7, -133.8), "path2_1": (-17, 204), "path2_4": (189, -6), "path3": (-51.9,127.3)
                            , "path10": (38.7, -8), "path12": (199, -208), "path13": (128, -205.4), "path14": (38, -207),
                           "path16": (-118, -137), "path18": (-34.9, -135.7), "path20": (-38, -199), "path22": (40.8, -196.4),
                            "path23": (112.2, -194.8), "path32": (196, 62)}
        self.direction_longX = {"path1_4": "Go Straight", "path2_1": "Turn Left or Go Straight", "path2_4": "Turn Right",
                                "path3": "Turn Right", "path10": "Turn Right", "path12": "Turn Left", "path13": "Turn Left",
                                "path14": "Turn Left", "path16": "Go Straight", "path18": "Go Straight", "path20": "Turn Right"
                                , "path22": "Turn Right", "path23": "Turn Right", "path32": "Turn Left"}

        self.path_narrow = {"path6": (-118, 0), "path7": (-52.5, 0.3), "path27": (105.8, -76), "path28": (115, -73)}
        self.direction_narrow = {"path1": "Go Straight", "path7": "Turn Left", "path27": "Turn Right", "path28": "Turn Left"}

        self.path_special = {"path16": (-113, -136)}
        self.direction_special = {"path16": "Go Straight"}

        self.navigate()

    def navigate(self):
        # ROAD TO Y (SHORT X)
        for key in self.path_longY:
            if self.path_longY[key][0] <= self.x <= self.path_longY[key][0]+4 and \
                    self.path_longY[key][1] <= self.y <= self.path_longY[key][1]+13:
                # return self.direction_longY[key]
                print(self.direction_longY[key])

            # ROAD TO X (SHORT Y)
        for key in self.path_longX:
            if self.path_longX[key][0] <= self.x <= self.path_longX[key][0]+13 and \
                    self.path_longX[key][1] <= self.y <= self.path_longX[key][1]+4:
                # return self.direction_longX[key]
                print(self.direction_longX[key])

            # Narrow Road
        for key in self.path_narrow:
            if self.path_narrow[key][0] <= self.x <= self.path_narrow[key][0]+13 and \
                    self.path_narrow[key][1] <= self.y <= self.path_narrow[key][1]+1:
                # return self.direction_narrow[key]
                print(self.direction_narrow[key])

            # Destination
        if 144.6 <= self.x <= 157.9 and -139.6 <= self.y <= -125.7:
            # return "destionation!"
            print("destination")

        # ==============================================================================
