#Header for use in a message filter
Header header

# The object will be attached with a fixed joint to this link
string link_name

#This contains the actual shapes and poses for the objects
#to be attached to the link
#If action is remove and objects is left blank, all objects
#attached to the link will be removed.
ObjectsInMap objects

# The set of links that the attached objects are allowed to touch
# by default - the link_name is included by default
string[] touch_links
