
#include "sh1106_oled.hpp"


void sh1106_oled::draw_char(point const & p, unsigned char c,GFXfont const * font,uint16_t color ) 
{

        c -= font->first;
        uint8_t const *bitmap = font->bitmap;
        GFXglyph const &glyph  = font->glyph[c];
        
        uint16_t bo = glyph.bitmapOffset;
        uint8_t const  w  = glyph.width;
        uint8_t const h  = glyph.height;
        int8_t const  xo = glyph.xOffset;
        int8_t const  yo = glyph.yOffset;
        uint8_t  bits = 0;
        uint8_t  bit = 0;

        for(uint8_t y=0; y<h; ++y) {
            for(uint8_t x=0; x<w; ++x) {
                if(!(bit++ & 7)) {
                    bits = bitmap[bo++];
                }
                if(bits & 0x80) {
                    set_pixel({p.x+xo+x, p.y+yo+y}, color);
                }
                bits <<= 1;
            }
        }
       // endWrite();

   // } // End classic vs custom font
}

