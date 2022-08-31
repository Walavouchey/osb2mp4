#pragma once

namespace sb {

    class Colour
    {
    public:
        Colour()
        {}
        Colour(double R, double G, double B)
            :
            R(R),
            G(G),
            B(B)
        {}
        double operator[](const int index) const
        {
            return index == 0 ? R : index == 1 ? G : index == 2 ? B : -1;
        };
        Colour operator+(const Colour& other) const
        {
            return Colour(R + other.R, G + other.G, B + other.B);
        };
        Colour operator-(const Colour& other) const
        {
            return Colour(R - other.R, G - other.G, B - other.B);
        };
        Colour operator*(const double other) const
        {
            return Colour(R * other, G * other, B * other);
        };
        double R;
        double G;
        double B;
    };

}
