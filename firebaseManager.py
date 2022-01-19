import pyrebase
MaxElements = 2

firebaseConfig = {
        'apiKey': "AIzaSyC1hNRnNx4zPS4lU9hRugfx3OlX7BD4sN8",
        'authDomain': "smartdispenser-9043d.firebaseapp.com",
        'databaseURL': "https://smartdispenser-9043d-default-rtdb.firebaseio.com",
        'projectId': "smartdispenser-9043d",
        'storageBucket': "smartdispenser-9043d.appspot.com",
        'messagingSenderId': "1059644042677",
        'appId': "1:1059644042677:web:fe599989918df767557dfc",
        'measurementId': "G-1V8PC6SFH1"
    }

class FireBaseManager:
        def __init__(self):
            # self.motor1 = None
            self.amount = 0
            self.amountList = [None] * MaxElements
        def Connect(self):
            try:
                firebase = pyrebase.initialize_app(firebaseConfig)
                db = firebase.database()
                parent = db.child("elements")
                temp = parent.get().val()
                self.amount = temp.get("amount")
                Ready = False
                for i in range(self.amount):
                    cur_user = "elements/element" + str(i)
                    # users_list.append(db.child(cur_user).get())
                    user = db.child(cur_user).get()
                    cur_amount = user.val().get("quantity")
                    if cur_amount > 0 :
                        Ready = True
                    self.amountList[i] = cur_amount  
                return Ready
            except:
                return False  
            
# if __name__ == "__main__":
    
#     fb = FireBaseManager()
#     RES = fb.Connect()
#     for element in fb.amountList:
#         print(element)