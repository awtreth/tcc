#ifndef IJSON_OBJECT_H
#define IJSON_OBJECT_H


class IJsonObject {


    public:

    virtual void fromJsonFile(const char* filePath) = 0;

    virtual void toJsonFile(const char* filePath) = 0;

};



#endif

