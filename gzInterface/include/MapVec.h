#ifndef MAPVEC_H
#define MAPVEC_H

#include <map>
#include <vector>
#include <string>

template <class T> class MapVec {
    private:

    std::map<std::string,int> nameMap;
    std::vector<T> elements;

    public:

    MapVec() {
        nameMap = std::map<std::string,int>();
        elements = std::vector<T>();
    }

    MapVec(std::vector<std::string> const names, const std::vector<T>& _elements) : MapVec(){
        elements = _elements;
        for(unsigned int i; i < elements.size(); i++)
            nameMap[names[i]] = i;
    }

    void add(std::string const name, T const& element) {
        nameMap[name] = elements.size();
        elements.push_back(element);
    }

    T get(int const idx) const{
        return elements[idx];
    }

    T get(const char* name) const{
        return elements[nameMap.at(name)];
    }

    void set(std::string const name, T const& element) {
        elements[nameMap[name]] = element;
    }

    void set(int const idx, T const& element) {
        elements[idx] = element;
    }

    unsigned int size() const {
        return elements.size();
    }

    std::vector<T> getElements() const{
        return elements;
    }

};



#endif
