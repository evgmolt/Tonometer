const uint16_t gsm[][29] = {
{0xFFFF,0xFFFF,0xFFFF,0xDFFF,0x38C6,0x4108,0x0C63,0xFFFF,0xFFFF,0xDFFF,0xFFFF,0xFFFF,0xDFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x0421,0x2421,0xFFFF,0xFFFF,0xFFFF,0xDFFF,0xFFFF,},
{0xFFFF,0xDFFF,0xFFFF,0x96B5,0x0000,0x0000,0x0000,0x1CE7,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xBEF7,0xFFFF,0x3084,0x0000,0x0000,0x0000,0xFFFF,0xFFFF,0xFFFF,0xDFFF,},
{0xFFFF,0xFFFF,0xDBDE,0x4108,0x0000,0x0000,0x55AD,0xDFFF,0xFFFF,0xDFFF,0xFFFF,0xDFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDFFF,0xFFFF,0xFFFF,0xFFFF,0xDFFF,0x494A,0x0000,0x2000,0x0000,0xDFFF,0xFFFF,0xFFFF,},
{0xFFFF,0xFFFF,0x0000,0x0000,0x2000,0xB294,0xDFFF,0xFFFF,0x3CE7,0x6529,0x2C63,0xFFFF,0xBEF7,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC739,0x2842,0xFFFF,0xFFFF,0xFFFF,0xA631,0x0000,0x0000,0xAA52,0xDFFF,0xFFFF,},
{0xFFFF,0x9294,0x2000,0x2000,0xA631,0xFFFF,0xFFFF,0x3CE7,0x0000,0x0000,0x0000,0x79CE,0xFFFF,0xDFFF,0xFFFF,0xFFFF,0xFFFF,0x2C63,0x0000,0x0000,0x6108,0xFFFF,0xFFFF,0xFFFF,0x0000,0x0000,0x0000,0x9EF7,0xFFFF,},
{0xFFFF,0x0000,0x0000,0x0000,0x9EF7,0xFFFF,0xFFFF,0x0000,0x0000,0x0000,0x8E73,0xFFFF,0xDFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x0421,0x0000,0x0000,0x694A,0xFFFF,0xFFFF,0x9294,0x0000,0x2000,0xEB5A,0xFFFF,},
{0x3CE7,0x0000,0x0000,0x2421,0xDFFF,0xDFFF,0x718C,0x2000,0x0000,0xC739,0xFFFF,0xFFFF,0xFFFF,0x96B5,0x718C,0xDFFF,0xDFFF,0xFFFF,0xFFFF,0x0000,0x0000,0x0000,0x3CE7,0xFFFF,0xFFFF,0x0000,0x0000,0x0000,0xFFFF,},
{0x14A5,0x0000,0x0000,0xCF7B,0xFFFF,0xFFFF,0x4529,0x0000,0x0000,0x3CE7,0xFFFF,0xFFFF,0xA631,0x0000,0x0000,0x0000,0xFBDE,0xFFFF,0xFFFF,0x718C,0x0000,0x0000,0x1084,0xFFFF,0xDFFF,0x2421,0x0000,0x0000,0xBEF7,},
{0x3084,0x0000,0x0000,0x34A5,0xFFFF,0xDFFF,0x0000,0x2000,0x2000,0xFFFF,0xFFFF,0x9AD6,0x0000,0x0000,0x0000,0x0000,0xC739,0xFFFF,0xFFFF,0xFBDE,0x0000,0x0000,0x8A52,0xBEF7,0xFFFF,0x694A,0x0000,0x0000,0xFBDE,},
{0x518C,0x2000,0x2000,0xF39C,0xFFFF,0xFFFF,0x0000,0x0000,0x0000,0xFFFF,0xFFFF,0xD7BD,0x0000,0x0000,0x0000,0x0000,0x2421,0xDFFF,0xFFFF,0x59CE,0x0000,0x0000,0xAA52,0xFFFF,0xFFFF,0x0842,0x0000,0x0000,0xFBDE,},
{0x75AD,0x0000,0x0000,0x6D6B,0xDFFF,0xFFFF,0xC739,0x0000,0x0000,0x9AD6,0xFFFF,0xFFFF,0x0000,0x0000,0x0000,0x0000,0x34A5,0xFFFF,0xDFFF,0xAE73,0x0000,0x0000,0xB294,0xFFFF,0xFFFF,0x8210,0x2000,0x0000,0xDFFF,},
{0xDFFF,0x0000,0x0000,0x8210,0xFFFF,0xDFFF,0x96B5,0x0000,0x0000,0x8210,0xFFFF,0xFFFF,0xFFFF,0x2C63,0x494A,0xF7BD,0xFFFF,0xFFFF,0xDFFF,0x2000,0x0000,0x0000,0xDFFF,0xDFFF,0xFFFF,0x0000,0x0000,0x8210,0xFFFF,},
{0xFFFF,0xC318,0x0000,0x0000,0x9AD6,0xDFFF,0xFFFF,0x0421,0x0000,0x0000,0x6529,0xDFFF,0xFFFF,0xFFFF,0xDFFF,0xFFFF,0xFFFF,0x7DEF,0x0000,0x0000,0x0000,0xEF7B,0xFFFF,0xFFFF,0x6D6B,0x2000,0x0000,0xCF7B,0xBEF7,},
{0xBEF7,0xD7BD,0x4108,0x0000,0x8210,0xFFFF,0xDFFF,0xFFFF,0x0000,0x0000,0x0000,0x9AD6,0xDFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x4D6B,0x0000,0x0000,0x2842,0xFFFF,0xFFFF,0xBEF7,0x0000,0x0000,0x0000,0xFFFF,0xFFFF,},
{0xFFFF,0xDFFF,0x0421,0x2000,0x0000,0xAA52,0xFFFF,0xFFFF,0xFFFF,0x8E73,0x34A5,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEF7B,0x718C,0xFFFF,0xFFFF,0xFFFF,0x4108,0x2000,0x0000,0x3084,0xFFFF,0xFFFF,},
{0xDFFF,0xFFFF,0xFFFF,0x0000,0x0000,0x0000,0x8A52,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x6108,0x0000,0x0000,0x4529,0xFFFF,0xFFFF,0xDFFF,},
{0xFFFF,0xDFFF,0xBFFF,0x9EF7,0x0000,0x0000,0x2000,0x3CDF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEF7B,0x0000,0x0000,0xA210,0xFFFF,0xFFFF,0xBEF7,0xFFFF,},
{0xBEF7,0xFFFF,0xDFFF,0xBFFF,0xBEFF,0x8952,0x139D,0xFEF7,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDFFF,0xFFFF,0xFFFF,0xFFFF,0x4D6B,0x6D6B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDFFF,},
};