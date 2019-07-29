
/*! 1D cubic bezier function, over interval t in [0,1] */
struct Bezier {
  /*! bezier coefficients */
  float b0, b1, b2, b3;
};

inline float eval(const Bezier &bezier, const float t)
{
  print("eval %\n",t);
  const float b00 = bezier.b0;
  const float b01 = bezier.b1;
  const float b02 = bezier.b2;
  const float b03 = bezier.b3;

  // print("b0\n %\n %\n %\n %\n",b00,b01,b02,b03);
  
  const float b10 = (1.f-t)*b00 + t*b01;
  const float b11 = (1.f-t)*b01 + t*b02;
  const float b12 = (1.f-t)*b02 + t*b03;
 
  // print("b1\n %\n %\n %\n",b10,b11,b12);

  const float b20 = (1.f-t)*b10 + t*b11;
  const float b21 = (1.f-t)*b11 + t*b12;

  // print("b2\n %\n %\n",b20,b21);
  
  const float b30 = (1.f-t)*b20 + t*b21;
  
  // print("b3\n %\n",b30);
  return b30;
}

inline Bezier sub(const Bezier &bezier, float isoValue)
{
  Bezier res;
  res.b0 = bezier.b0 - isoValue;
  res.b1 = bezier.b1 - isoValue;
  res.b2 = bezier.b2 - isoValue;
  res.b3 = bezier.b3 - isoValue;
  return res;
}

inline float min(const Bezier &b)
{
  return min(min(b.b0,b.b1),min(b.b2,b.b3));
}

inline float max(const Bezier &b)
{
  return max(max(b.b0,b.b1),max(b.b2,b.b3));
}

inline void subdivide(Bezier &front, Bezier &back, const Bezier &bezier)
{
  const float b00 = bezier.b0;
  const float b01 = bezier.b1;
  const float b02 = bezier.b2;
  const float b03 = bezier.b3;

  // print("b0\n %\n %\n %\n %\n",b00,b01,b02,b03);
  
  const float b10 = 0.5f*b00 + 0.5f*b01;
  const float b11 = 0.5f*b01 + 0.5f*b02;
  const float b12 = 0.5f*b02 + 0.5f*b03;
 
  // print("b1\n %\n %\n %\n",b10,b11,b12);

  const float b20 = 0.5f*b10 + 0.5f*b11;
  const float b21 = 0.5f*b11 + 0.5f*b12;

  // print("b2\n %\n %\n",b20,b21);

  const float b30 = 0.5f*b20 + 0.5f*b21;

  // print("b3\n %\n",b30);
  front.b0 = b00;
  front.b1 = b10;
  front.b2 = b20;
  front.b3 = b30;

  back.b3 = b03;
  back.b2 = b12;
  back.b1 = b21;
  back.b0 = b30;
}

