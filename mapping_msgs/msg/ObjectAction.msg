#Puts the object into the environment
#or updates the object if already added
byte ADD=0

#Removes the object from the environment entirely
byte REMOVE=1

#Only valid in attached object message
#Takes an attached object, detaches from the attached link
#But adds back in as regular object
byte DETACH_AND_ADD_AS_OBJECT=2

#Only valid with attached object message
#Takes current object in the environment and removes it as
#a regular object
byte ATTACH_AND_REMOVE_AS_OBJECT=3

# Byte code for action
byte action
