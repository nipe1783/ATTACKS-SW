# pragma once

class Blob{
    public:
        // methods
        Blob(int x, int y, int width, int height, int area);
        Blob();
        ~Blob();

        // fields
        int x;
        int y;
        int width;
        int height;
        int area;
};