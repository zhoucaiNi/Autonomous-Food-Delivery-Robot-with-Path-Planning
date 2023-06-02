


class Delivery:
    def __init__(self, dorms, resturants, start):
        self.dorms = dorms
        self.restaurants = resturants
        self.start = start
        
    def getPoints(self, nameList):
        deliveryList = [self.start]
        for i in nameList:
          if  i in self.dorms:
              deliveryList.append(self.dorms[i])
          elif i in self.restaurants:
              deliveryList.append(self.restaurants[i])
        return deliveryList
    
    def pathToNames(self, nameList,optimal_path):
        path_format = []
        for i in optimal_path:
          if i == 0:
              path_format.append("start")
          else:
                path_format.append(nameList[i-1])
        return path_format

    def pointsToCells(self, resolution, target):
        if target == "dorms":
          currDict = self.dorms.copy()
        elif target == "restaurants":
          currDict = self.restaurants.copy()
        else:
          print("invalid target")
          return
        
        for key in currDict.keys():
            currDict[key] = (currDict[key][0] / resolution, currDict[key][1] / resolution)
        return currDict
    
    def pathToPoints(self, path_format):
        deliveryList = []
        for i in path_format:
          if  i in self.dorms:
              deliveryList.append(self.dorms[i])
          elif i in self.restaurants:
              deliveryList.append(self.restaurants[i])
          elif i == "start":
             deliveryList.append(self.start)
        return deliveryList