
#include <cstring>
#include "sh1106_oled.hpp"


void sh1106_oled::draw_text(point const & p,const char* text,GFXfont const * font,uint16_t color )
{
   if ( (text != nullptr) && (font != nullptr) ){

      auto const text_len = strlen(text); // n.b. length not including null
      uint8_t const *bitmap = font->bitmap;
      auto pos = p;
      for ( uint32_t i = 0; i < text_len; ++i){
          char const c = text[i];
          draw_char(pos,c,font,color);
          char const idx = c - font->first;
          GFXglyph const &glyph  = font->glyph[idx];
          pos.x += glyph.xAdvance;
      }
   }
}

