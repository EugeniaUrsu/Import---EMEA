Ќё$џўџB X I R A A A L X M Q Q 3 B G I V N K N 
 T e m p l a t e :   N e w J o b S u b f o r m 
 D a t e   C r e a t e d :   ! 2 0 1 1 , 0 4 , 2 2 , 1 4 , 3 4 , 2 1 
 D a t e   M o d i f i e d :   ! 2 0 1 6 , 0 4 , 0 8 , 1 4 , 1 3 , 2 6 
 P o w e r D B   r e v :   1 2 . t r u n k    џўџT e m p l a t e G U I D    џўџB X I R A A A L X M Q Q 3 B G I V N K N џўџN a m e    џўџN e w J o b S u b f o r m џўџ
D e f i n i t i o n    С optform          SAVDOUT}s65>?џўџ   А@  0A                =                         џџ  OOptDrawCtrlScriptmE#џўџO n   I n i t i a l   U p d a t e џўџc a l l   N e w J o b I n i t ( )     mE#џўџ	O n   U p d a t e џўџAc a l l   N e w J o b O n U p d a t e ( )  
 c a l l   S e t V a r ( " S c r i p t T i m e o u t M s " ,   C I n t ( - 1 ) )  
      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџџЖkD i m   g _ U I S t a c k  
 S e t   g _ U I S t a c k   =   n e w   S t a c k  
  
 c l a s s   S t a c k  
        
         p r i v a t e   s u b   c l a s s _ I n i t i a l i z e ( )  
                 c a l l   a L o g ( " c l a s s _ I n i t i a l i z e " )  
                 P o s   =   G e t V a r ( " S t a c k P o s " )  
                  
                 I f   I s B l a n k S t r i n g ( P o s )   T h e n  
                         c a l l   S e t V a r ( " S t a c k P o s " ,   0 )  
                 E n d   I f  
                  
         e n d   s u b  
          
         p u b l i c   s u b   P u s h ( I t e m )          
                 P o s   =   G e t V a r ( " S t a c k P o s " )  
                  
                 P o s   =   P o s   +   1  
                 c a l l   S e t N d x V a r ( " S t a c k D a t a " ,   P o s ,   I t e m )  
                 c a l l   S e t V a r ( " S t a c k P o s " ,   P o s )  
                  
         e n d   s u b  
          
         p u b l i c   f u n c t i o n   P o p ( )          
                 I t e m   =   P e e k ( )  
                  
                 P o s   =   G e t V a r ( " S t a c k P o s " )  
                 I f   P o s   >   0   T h e n  
                         P o s   =   P o s   -   1  
                         c a l l   S e t V a r ( " S t a c k P o s " ,   P o s )  
                 E n d   I f  
                  
                 P o p   =   I t e m  
         e n d   f u n c t i o n  
          
         p u b l i c   f u n c t i o n   P e e k ( )  
                 P o s   =   G e t V a r ( " S t a c k P o s " )  
                  
                 I f   P o s   >   0   T h e n  
                         I t e m   =   G e t N d x V a r ( " S t a c k D a t a " ,   P o s )  
                 E l s e  
                         I t e m   =   " "  
                 E n d   I f  
                 P e e k   =   I t e m  
         e n d   f u n c t i o n  
          
         p u b l i c   s u b   P r i n t S t a c k ( )  
                 S t r   =   " "  
                  
                 P o s   =   G e t V a r ( " S t a c k P o s " )  
                 f o r   i   =   P o s   t o   1   S t e p   - 1  
                         S t r   =   S t r   &   G e t N d x V a r ( " S t a c k D a t a " ,   i )   &   C h r ( 1 3 )   &   C h r ( 1 0 )  
                 n e x t  
                 c a l l   a L o g ( " S t a c k = "   &   S t r )  
         e n d   s u b  
          
         p u b l i c   s u b   C l e a r ( )  
                 c a l l   S e t V a r ( " S t a c k P o s " ,   0 )  
         e n d   s u b  
          
 e n d   c l a s s  
  
 s u b   N e w J o b I n i t ( )                
          
         F i l e N a m e   = " C : \ m o b i l e \ P D B _ I n s p e c t i o n . t x t "  
         s t r N e w N a m e   =   " C : \ m o b i l e \ P D B _ I n s p e c t i o n . o l d "  
          
         S e t   o b j F S O   =   C r e a t e O b j e c t ( " S c r i p t i n g . F i l e S y s t e m O b j e c t " )  
          
         i f   ( o b j F S O . F i l e E x i s t s ( F i l e N a m e ) )   t h e n  
                  
                 S e t   o b j F i l e   =   o b j F S O . O p e n T e x t F i l e ( F i l e N a m e ,   1 )  
          
                 D o   U n t i l   o b j F i l e . A t E n d O f S t r e a m  
                         s t r L i n e   =   o b j F i l e . R e a d L i n e  
                         c a l l   a l o g ( " s t r L i n e :   "   +   s t r L i n e )  
                         i f   (   L e f t ( s t r L i n e , 2 )   < >   " / / "   )   t h e n  
                                 i f   ( I n S t r ( s t r L i n e , " T a g = " ) )   t h e n  
                                         T a g I D   =   M i d ( s t r L i n e , 5 )  
                                         c a l l   S e t V a r ( " T a g I D " , T a g I D )  
                                 e l s e i f   ( I n S t r ( s t r L i n e , " T i c k e t = " ) )   t h e n  
                                         T i c k e t   =   M i d ( s t r L i n e , 8 )  
                                         c a l l   S e t V a r ( " T i c k e t " , T i c k e t )  
                                 e l s e i f   ( I n S t r ( s t r L i n e , " R e p o r t = " ) )   t h e n  
                                         R e p o r t T y p e   =   M i d ( s t r L i n e , 8 )  
                                         s e l e c t   c a s e   R e p o r t T y p e  
                                         c a s e   " 1 0 7 7 0 "  
                                                 F o r m G U I D   =   " B X I R A A A I 3 N H J 4 U S Q Y I S N "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 0 )  
                                         c a s e   " 2 0 1 0 0 "  
                                                 F o r m G U I D   =   " A U Q 3 A A A I R S U 2 J A S T 4 X S P "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 1 )  
                                         c a s e   " 8 0 2 0 0 "  
                                                 F o r m G U I D   =   " A V Y 3 A A A C Z B F P 2 7 P U 6 F 2 S "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 2 )  
                                         c a s e   " 8 0 1 0 0 "  
                                                 F o r m G U I D   =   " A V Y 3 A A A N D G K P J Q Y F C X S T "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 3 )  
                                         c a s e   " 8 0 3 0 0 "  
                                                 F o r m G U I D   =   " A V Y 3 A A A A D G 4 L K R G I E F C T "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 4 )  
                                         c a s e   " 1 0 7 7 1 "  
                                                 F o r m G U I D   =   " A V Y 3 A A A D S V 7 S H 2 M B M O C S "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 5 )  
                                         c a s e   " T H E R M A L   P M V "  
                                                 F o r m G U I D   =   " A 2 Y W A A A G N Z 2 B I S 6 P 7 U C U "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 6 )  
                                         c a s e   " 8 0 8 0 0   U P S "  
                                                 F o r m G U I D   =   " A 2 Y W A A A G N Z 2 B I S 6 P 7 U C U "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 7 )  
                                         c a s e   " E H S   S i t e   S u r v e y "  
                                                 F o r m G U I D   =   " A V Y 3 A A A O J T 2 J Q 6 B C E B S X "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 8 )  
  
                                         e n d   s e l e c t  
                                 e n d   i f  
                         e n d   i f  
                 L o o p  
                        
                 o b j F i l e . C l o s e  
                 S e t   o b j F i l e   =   N o t h i n g  
                  
                 i f   ( L o o k u p T a g ( T a g I D ,   S i t e I D ,   C o m p a n y ,   R e l a y G U I D ,   D e v i c e G U I D ,   R e g i o n G U I D ) )   A n d   ( F o r m G U I D   =   D e v i c e G U I D )   T h e n  
                         c a l l   S e t V a r ( " T a g I D " , T a g I D )  
                         c a l l   a l o g ( " T a g I D =   "   +   T a g I D )  
                         c a l l   S e t V a r ( " T i c k e t " , T i c k e t )  
                         c a l l   a L o g ( " R e l a y G U I D = "   &   R e l a y G U I D   &   " , C o m p a n y = "   &   C o m p a n y   &   " , S i t e I D = "   &   S i t e I D   &   " , F o r m I D = "   &   D e v i c e G U I D )  
                         s e l e c t   c a s e   R e p o r t T y p e  
                         c a s e   " 1 0 7 7 0 "  
                                 F o r m G U I D   =   " B X I R A A A I 3 N H J 4 U S Q Y I S N "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 0 )  
                         c a s e   " 2 0 1 0 0 "  
                                 F o r m G U I D   =   " A U Q 3 A A A I R S U 2 J A S T 4 X S P "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 1 )  
                         c a s e   " 8 0 2 0 0 "  
                                 F o r m G U I D   =   " A V Y 3 A A A C Z B F P 2 7 P U 6 F 2 S "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 2 )  
                         c a s e   " 8 0 1 0 0 "  
                                 F o r m G U I D   =   " A V Y 3 A A A N D G K P J Q Y F C X S T "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 3 )  
                         c a s e   " 8 0 3 0 0 "  
                                 F o r m G U I D   =   " A V Y 3 A A A A D G 4 L K R G I E F C T "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 4 )  
                         c a s e   " 1 0 7 7 1 "  
                                 F o r m G U I D   =   " A V Y 3 A A A D S V 7 S H 2 M B M O C S "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 5 )  
                         c a s e   " T H E R M A L   P M V "  
                                 F o r m G U I D   =   " A 2 Y W A A A G N Z 2 B I S 6 P 7 U C U "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 6 )  
                         c a s e   " U P S "  
                                 F o r m G U I D   =   " A 2 Y W A A A O M T P Y L 2 I H Q J S W "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 7 )  
                         c a s e   " E H S   S i t e   S u r v e y "  
                                 F o r m G U I D   =   " A V Y 3 A A A O J T 2 J Q 6 B C E B S X "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 8 )  
  
                         e n d   s e l e c t  
                          
                         i f   o b j F S O . F i l e E x i s t s ( s t r N e w N a m e )   t h e n  
                                 o b j F S O . D e l e t e F i l e   s t r N e w N a m e  
                         e n d   i f  
                          
                         o b j F S O . M o v e F i l e   F i l e N a m e ,   s t r N e w N a m e  
                 e l s e  
                         c a l l   a l o g ( " R e p o r t   T y p e   f r o m   f i l e :   "   &   F o r m G U I D )  
                         c a l l   a l o g ( " R e p o r t   T y p e   f r o m   T a g :   "   &   D e v i c e G U I D )  
                         c a l l   S e t V a r ( " R e p o r t T y p e " , " " )  
                         c a l l   a l o g ( " T a g   n o t   f o u n d   o r   i n c o r r e c t   r e p o r t   t y p e   i n   P D B _ I n s p e c t i o n . t x t :   D e l e t i n g   F i l e " )  
                         o b j F S O . D e l e t e F i l e   F i l e n a m e  
                 e n d   i f  
         e n d   i f  
         S e t S c r e e n ( " L o o k u p " )  
          
 e n d   s u b  
  
 s u b   S e t S c r e e n ( N e w S c r e e n )        
         c a l l   a L o g ( " S e t S c r e e n ( "   &   N e w S c r e e n   &   " )   S T A R T   - - - - - - - - - - - " )  
         c a l l   S e t V a r ( " U I S t a t e " ,   N e w S c r e e n )  
         c a l l   g _ U I S t a c k . P u s h ( N e w S c r e e n )  
         c a l l   g _ U I S t a c k . P r i n t S t a c k ( )  
         c a l l   a L o g ( " S e t S c r e e n ( "   &   N e w S c r e e n   &   " )   E N D   - - - - - - - - - - - " )  
 e n d   s u b  
  
 s u b   G o B a c k ( )  
         c a l l   a L o g ( " G o B a c k ( "   &   N e w S c r e e n   &   " )   S T A R T   - - - - - - - - - - - " )  
         g _ U I S t a c k . P o p ( )  
         S c r e e n   =   g _ U I S t a c k . P e e k ( )  
          
         c a l l   S e t V a r ( " U I S t a t e " ,   S c r e e n )                  
         c a l l   a L o g ( " G o B a c k ( "   &   N e w S c r e e n   &   " )   E N D   - - - - - - - - - - - " )  
 e n d   s u b  
  
  
 s u b   N e w J o b O n U p d a t e ( )  
         U I S t a t e   =   G e t V a r ( " U I S t a t e " )  
         c a l l   a L o g ( " N e w J o b O n U p d a t e :   U I S t a t e   =   "   &   U I S t a t e )  
          
         S u b F o r m s   =   A r r a y ( " L o o k u p " ,   " C o n f i r m J o b C r e a t e " ,   " C o n f i r m T a g C r e a t e " ,   " L o o k u p S i t e " ,   " L o o k u p S i t e B y C u s t o m e r " ,   " C o n f i r m S i t e C r e a t e " ,   " C r e a t e N e w " )  
         H i d d e n T a g s   =   " "  
          
         f o r   i   =   0   t o   U B o u n d ( S u b F o r m s )  
                 S u b f o r m S u f f i x   =   S u b F o r m s ( i )  
                 S u b f o r m N a m e   =   " _ _ N e w J o b S u b f o r m "   &   S u b f o r m S u f f i x  
                  
                 i f   S u b f o r m S u f f i x   =   U I S t a t e   t h e n  
                         c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   S u b f o r m N a m e ,   " V i s i b l e   1 : S e n d T o F r o n t   1 " )  
                         c a l l   S e t V a r ( " M o d a l S u b f o r m " ,   S u b f o r m N a m e )  
                 e l s e  
                         H i d d e n T a g s   =   H i d d e n T a g s   &   S u b f o r m N a m e   &   " , "  
                 e n d   i f                  
         n e x t  
         c a l l   a L o g ( " H i d d e n T a g s   =   "   &   H i d d e n T a g s )  
         c a l l   S e t V i s i b l e ( H i d d e n T a g s ,   0 )  
          
          
          
          
         s e l e c t   c a s e   U I S t a t e  
                 c a s e   " L o o k u p "  
                         c a l l   V a l i d a t e L o o k u p F i e l d s ( F a l s e )  
                 c a s e   " C o n f i r m J o b C r e a t e "  
                 c a s e   " C o n f i r m T a g C r e a t e "  
                 c a s e   " L o o k u p S i t e "  
                         c a l l   V a l i d a t e S i t e F i e l d s ( F a l s e )                  
                 c a s e   " L o o k u p S i t e B y C u s t o m e r "  
                 c a s e   " C o n f i r m S i t e C r e a t e "  
                 c a s e   " C o n f i r m T a g C r e a t e "  
         e n d   s e l e c t  
          
 e n d   s u b  
  
 f u n c t i o n   V a l i d a t e L o o k u p F i e l d s ( V a l i d a t e R e q u i r e d )  
         b V a l i d   =   t r u e  
          
         b V a l i d   =   b V a l i d   A N D   V a l i d a t e N u m e r i c F i e l d ( " T i c k e t " ,   7 ,   V a l i d a t e R e q u i r e d )  
         b V a l i d   =   b V a l i d   A N D   V a l i d a t e N u m e r i c F i e l d ( " T a g I D " ,   7 ,   V a l i d a t e R e q u i r e d )  
                  
         V a l i d a t e L o o k u p F i e l d s   =   b V a l i d          
 e n d   f u n c t i o n  
  
 f u n c t i o n   V a l i d a t e L o o k u p F i e l d s T a g I D ( V a l i d a t e R e q u i r e d )  
     '     b V a l i d   =   t r u e  
          
     '     b V a l i d   =   b V a l i d   A N D   V a l i d a t e N u m e r i c F i e l d R a n g e ( " T a g I D " ,   4 , 7 ,   V a l i d a t e R e q u i r e d )  
                  
       '   V a l i d a t e L o o k u p F i e l d s   =   b V a l i d          
 e n d   f u n c t i o n  
  
 f u n c t i o n   V a l i d a t e S i t e F i e l d s ( V a l i d a t e R e q u i r e d )  
         b V a l i d   =   t r u e  
          
         b V a l i d   =   b V a l i d   A N D   V a l i d a t e N u m e r i c F i e l d ( " S i t e I D " ,   6 ,   V a l i d a t e R e q u i r e d )  
          
         V a l i d a t e S i t e F i e l d s   =   b V a l i d          
 e n d   f u n c t i o n  
  
 f u n c t i o n   V a l i d a t e C u s t o m e r F i e l d ( )  
         b V a l i d   =   t r u e  
          
         b V a l i d   =   b V a l i d   A N D   V a l i d a t e I s N o t B l a n k F i e l d ( " C u s t o m e r " )  
          
         V a l i d a t e C u s t o m e r F i e l d   =   b V a l i d          
 e n d   f u n c t i o n  
  
 f u n c t i o n   V a l i d a t e R e p o r t T y p e F i e l d ( )  
         b V a l i d   =   t r u e  
          
         b V a l i d   =   b V a l i d   A N D   V a l i d a t e I s N o t B l a n k F i e l d ( " R e p o r t T y p e " )  
          
         V a l i d a t e R e p o r t T y p e F i e l d   =   b V a l i d          
 e n d   f u n c t i o n  
  
 f u n c t i o n   V a l i d a t e I s N o t B l a n k F i e l d ( F i e l d N a m e )  
                  
         V a l i d   =   t r u e  
          
         V a l u e   =   G e t V a r ( F i e l d N a m e )  
          
         V a l u e   =   T r i m ( V a l u e )  
         V a l i d   =   V a l i d   A N D   N O T   I s B l a n k S t r i n g ( V a l u e )   A N D   V a l u e   < >   " - 1 "  
          
         I f   V a l i d   T h e n  
                 c a l l   S e t V a r ( F i e l d N a m e   &   " E r r " ,   " " )  
         E l s e  
               '   E r r M s g   =   F i e l d N a m e   &   "   i s   r e q u i r e d "            
                 E r r M s g   =   " T h i s   f i e l d   i s   r e q u i r e d "           ' K i r a  
                 c a l l   S e t V a r ( F i e l d N a m e   &   " E r r " ,   E r r M s g )  
         E n d   I f  
  
         V a l i d a t e I s N o t B l a n k F i e l d   =   V a l i d  
          
 e n d   f u n c t i o n  
  
 f u n c t i o n   V a l i d a t e N u m e r i c F i e l d ( F i e l d N a m e ,   L e n g t h ,   V a l i d a t e R e q u i r e d )  
          
         D i m   R e g E x  
                  
         V a l i d   =   t r u e  
          
         V a l u e   =   G e t V a r ( F i e l d N a m e )  
          
         V a l u e   =   T r i m ( V a l u e )  
         V a l L e n g t h   =   L e n ( V a l u e )  
         E r r M s g   =   " "  
          
         c a l l   a L o g ( " V a l i d a t e N u m e r i c F i e l d : "   &   F i e l d N a m e   &   " , v a l u e = "   &   V a l u e   &   " , L e n g t h = "   &   V a l L e n g t h   &   " , r e q = "   &   V a l i d a t e R e q u i r e d )  
          
         I f   V a l L e n g t h   =   0   T h e n  
          
                 '   R e q u i r e d  
                 c a l l   a L o g ( " B e f o r e   V a l i d a t e R e q u i r e d   : "   &   V a l i d a t e R e q u i r e d )  
                 c a l l   a L o g ( " V a l i d a t e   R e q u i r e d " )  
                 V a l i d   =   F a l s e  
                  
                 R e q u i r e d E r r M s g   =   F i e l d N a m e   &   "   i s   r e q u i r e d "  
                  
                 I f   V a l i d a t e R e q u i r e d   O r   G e t V a r ( F i e l d N a m e   &   " E r r " )   =   R e q u i r e d E r r M s g   T h e n                          
                         E r r M s g   =   R e q u i r e d E r r M s g  
                 E n d   I f  
         E l s e  
                 S e t   R e g E x   =   N e w   R e g E x p  
                 R e g E x . P a t t e r n   =   " ^ \ d { "   &   L e n g t h   &   " } $ "  
                 V a l i d   =   R e g E x . T e s t ( V a l u e )  
                 I f   N o t   V a l i d   T h e n  
                         ' E r r M s g   =   F i e l d N a m e   &   "   s h o u l d   b e   n u m e r i c   a n d   "   &   L e n g t h   &   "   d i g i t s   l o n g . "  
                         ' E r r M s g   =   " T h i s   f i e l d   s h o u l d   b e   n u m e r i c   a n d   "   &   L e n g t h   &   "   d i g i t s   l o n g . "  
                         E r r M s g   =   " M U S T   b e   "   &   L e n g t h   &   "   d i g i t   n o . ,   a d d   l e a d i n g   z e r o   i f   r e q u i r e d . "  
                 E n d   I f                  
         E n d   I f  
          
         c a l l   a L o g ( " V a l i d = "   &   V a l i d   &   " ,   E r r M s g   = "   &   E r r M s g )  
         I f   V a l i d   T h e n  
                 c a l l   S e t V a r ( F i e l d N a m e   &   " E r r " ,   " " )  
         E l s e  
                 c a l l   S e t V a r ( F i e l d N a m e   &   " E r r " ,   E r r M s g )  
         E n d   I f  
  
         V a l i d a t e N u m e r i c F i e l d   =   V a l i d  
          
 e n d   f u n c t i o n    
  
 f u n c t i o n   V a l i d a t e N u m e r i c F i e l d R a n g e ( F i e l d N a m e ,   S t a r t ,   E n d V a l u e ,   V a l i d a t e R e q u i r e d )  
          
         D i m   R e g E x  
                  
         V a l i d   =   t r u e  
          
         V a l u e   =   G e t V a r ( F i e l d N a m e )  
          
         V a l u e   =   T r i m ( V a l u e )  
         V a l L e n g t h   =   L e n ( V a l u e )  
         E r r M s g   =   " "  
          
         c a l l   a L o g ( " V a l i d a t e N u m e r i c F i e l d : "   &   F i e l d N a m e   &   " , v a l u e = "   &   V a l u e   &   " , L e n g t h = "   &   V a l L e n g t h   &   " , r e q = "   &   V a l i d a t e R e q u i r e d )  
          
         I f   V a l L e n g t h   =   0   T h e n  
          
                 '   R e q u i r e d  
                 c a l l   a L o g ( " B e f o r e   V a l i d a t e R e q u i r e d   : "   &   V a l i d a t e R e q u i r e d )  
                 c a l l   a L o g ( " V a l i d a t e   R e q u i r e d " )  
                 V a l i d   =   F a l s e  
                  
                 ' R e q u i r e d E r r M s g   =   F i e l d N a m e   &   "   i s   r e q u i r e d "  
                 R e q u i r e d E r r M s g   =   " T h i s   f i e l d   i s   r e q u i r e d "  
                  
                 I f   V a l i d a t e R e q u i r e d   O r   G e t V a r ( F i e l d N a m e   &   " E r r " )   =   R e q u i r e d E r r M s g   T h e n                          
                         E r r M s g   =   R e q u i r e d E r r M s g  
                 E n d   I f  
         E l s e  
                 S e t   R e g E x   =   N e w   R e g E x p  
                 R e g E x . P a t t e r n   =   " ^ \ d { "   &   S t a r t   &   " , "   &   E n d V a l u e   &   " } $ "  
                 V a l i d   =   R e g E x . T e s t ( V a l u e )  
                 I f   N o t   V a l i d   T h e n  
                         ' E r r M s g   =   F i e l d N a m e   &   "   s h o u l d   b e   n u m e r i c   a n d   "   &   S t a r t   &   " - "   &   E n d V a l u e   &   "   d i g i t s . "  
                         E r r M s g   =   " T h i s   f i e l d   s h o u l d   b e   n u m e r i c   a n d   "   &   S t a r t   &   " - "   &   E n d V a l u e   &   "   d i g i t s . "  
                 E n d   I f                  
         E n d   I f  
          
         c a l l   a L o g ( " V a l i d = "   &   V a l i d   &   " ,   E r r M s g   = "   &   E r r M s g )  
         I f   V a l i d   T h e n  
                 c a l l   S e t V a r ( F i e l d N a m e   &   " E r r " ,   " " )  
         E l s e  
                 c a l l   S e t V a r ( F i e l d N a m e   &   " E r r " ,   E r r M s g )  
         E n d   I f  
  
         V a l i d a t e N u m e r i c F i e l d R a n g e   =   V a l i d  
          
 e n d   f u n c t i o n    
  
 s u b   L o o k u p T i c k e t A n d T a g ( )  
         D i m   J o b G u i d  
         D i m   S i t e I D  
         D i m   C o m p a n y  
         D i m   R e l a y G u i d  
          
         c a l l   a L o g ( " B e f o r e " )  
         c a l l   S e t V a r ( " R e l a y G U I D " ,   E m p t y )  
         c a l l   S e t V a r ( " C u s t o m e r " ,   E m p t y )  
         c a l l   S e t V a r ( " S i t e I D " ,   E m p t y )  
         c a l l   S e t V a r ( " D e v i c e G U I D " ,   E m p t y )          
  
         V a l i d   =   V a l i d a t e L o o k u p F i e l d s ( t r u e )  
         c a l l   a L o g ( " L o o k u p T i c k e t A n d T a g   = "   &   V a l i d )  
          
          
         I f   V a l i d   T h e n  
                 T i c k e t   =   G e t V a r ( " T i c k e t " )  
                 T a g I D   =   G e t V a r ( " T a g I D " )  
                 c a l l   S e t V a r ( " U p d a t e T a g " ,   0 )          
                 J o b N u m   =   T i c k e t   &   " - "   &   T a g I D  
                 I f   L o o k u p R e p o r t ( T a g I D ,   D e v i c e G U I D )   t h e n                  
                         I f   L o o k u p J o b ( J o b N u m ,   J o b G u i d )   T h e n  
                                 '   O p e n   J o b  
                                 c a l l   A p p l i c a t i o n . O p e n J o b ( J o b G u i d )  
                                 c a l l   E x i t P o p u p ( )  
                         E l s e I f   L o o k u p T a g ( T a g I D ,   S i t e I D ,   C o m p a n y ,   R e l a y G U I D ,   D e v i c e G U I D ,   R e g i o n G U I D )   T h e n  
                                 c a l l   a L o g ( " R e l a y G U I D = "   &   R e l a y G u i d   &   " , C o m p a n y = "   &   C o m p a n y   &   " , S i t e I D = "   &   S i t e I D )  
                                 c a l l   S e t V a r ( " R e l a y G U I D " ,   R e l a y G U I D )  
                                 c a l l   S e t V a r ( " C u s t o m e r " ,   C o m p a n y )  
                                 c a l l   S e t V a r ( " S i t e I D " ,   S i t e I D )  
                                 c a l l   S e t V a r ( " R e g i o n G U I D " , R e g i o n G U I D )  
                                  
                                 '   L o o k u p   s i t e   a d d r e s s  
                                 C o m p a n y G u i d   =   L o a d S i t e A d d r e s s ( S i t e I D )  
                                  
                                 i f   N O T   I s B l a n k S t r i n g ( C o m p a n y G u i d )   t h e n  
                                         c a l l   S e t V a r ( " C o m p a n y G u i d " ,   C o m p a n y G u i d )  
                                 e n d   i f  
                                  
                                 c a l l   S e t S c r e e n ( " C o n f i r m J o b C r e a t e " )  
                         E l s e  
                                 c a l l   S e t S c r e e n ( " C o n f i r m T a g C r e a t e " )  
                         E n d   I f  
                 E l s e          
                         E r r M s g   =   " I n v a l i d   r e p o r t   t y p e "  
                         c a l l   S e t V a r ( " R e p o r t T y p e E r r " ,   E r r M s g )  
                         c a l l   S e t V a r ( " U p d a t e T a g " ,   1 )  
                 E n d   I f  
         E n d   I f  
          
 e n d   s u b  
  
 f u n c t i o n   L o o k u p R e p o r t ( T a g ,   B y R e f   D e v i c e G U I D )  
          
         b F o u n d   =   f a l s e  
          
         S q l   =   " S E L E C T   T O P   1   D e v i c e G U I D   F R O M   R e l a y   W H E R E   b I s D e l   =   0   A N D   O r g 4   =   ' "   &   T a g   &   " '   O r d e r   b y   M o d D a t e   d e s c "  
         c a l l   a L o g   ( " S Q L   =   "   &   S q l )  
         c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S q l ,   0 )  
          
         D e v i c e G U I D   =   G e t V a r ( " d b _ D e v i c e G U I D " )  
  
         R e p o r t T y p e   =   G e t V a r ( " R e p o r t T y p e " )  
         c a l l   a L o g ( " R e p o r t   T y p e   =   "   &   R e p o r t T y p e )  
         s e l e c t   c a s e   R e p o r t T y p e  
                 c a s e   0  
                         F o r m G U I D   =   " B X I R A A A I 3 N H J 4 U S Q Y I S N "  
                 c a s e   1  
                         F o r m G U I D   =   " A U Q 3 A A A I R S U 2 J A S T 4 X S P "  
                 c a s e   2  
                         F o r m G U I D   =   " A V Y 3 A A A C Z B F P 2 7 P U 6 F 2 S "  
                 c a s e   3  
                         F o r m G U I D   =   " A V Y 3 A A A N D G K P J Q Y F C X S T "  
                 c a s e   4  
                         F o r m G U I D   =   " A V Y 3 A A A A D G 4 L K R G I E F C T "  
                 c a s e   5  
                         F o r m G U I D   =   " A V Y 3 A A A D S V 7 S H 2 M B M O C S "  
                 c a s e   6  
                         F o r m G U I D   =   " A 2 Y W A A A G N Z 2 B I S 6 P 7 U C U "  
                 c a s e   7  
                         F o r m G U I D   =   " A 2 Y W A A A O M T P Y L 2 I H Q J S W "  
                 c a s e   8  
                         F o r m G U I D   =   " A V Y 3 A A A O J T 2 J Q 6 B C E B S X "  
                          
         e n d   s e l e c t          
         c a l l   a L o g ( " D e v i c e G U I D   =   "   &   D e v i c e G U I D )  
         c a l l   a L o g ( " F o r m G U I D   =   "   &   F o r m G U I D )  
         i f   (   D e v i c e G U I D   =   F o r m G U I D   )   O R   (   D e v i c e G U I D   =   " "   ) t h e n  
                 b F o u n d   =   t r u e  
         e n d   i f  
          
         L o o k u p R e p o r t   =   b F o u n d  
          
 e n d   f u n c t i o n  
  
 f u n c t i o n   L o o k u p J o b ( J o b N u m ,   B y R e f   J o b G u i d )  
          
         b F o u n d   =   f a l s e  
          
         '   T o D o :   R e s t r i c t   t o   j o b s   i n   u s e r s   r e g i o n   ( i f   s e t   i n   u s e r   a c c o u n t ) .  
         '   R e s t r i c t   t o   o w n   j o b s   ( i f   s e t   i n   u s e r   a c c o u n t )  
         S q l   =   " S E L E C T   J o b G u i d   F R O M   P d b J o b   W H E R E   J o b N u m b e r   =   ' "   &   J o b N u m   &   " '   A N D   b I s D e l   =   0 "  
         c a l l   a L o g ( " S Q L   =   "   &   S q l )  
         c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S q l ,   0 )  
          
         J o b G u i d   =   G e t V a r ( " d b _ J o b G u i d " )  
         c a l l   a L o g ( " J o b G u i d = "   &   J o b G u i d )  
         c a l l   a L o g ( " J o b N u m b e r = "   &   J o b N u m )  
         I f   N o t   I s B l a n k S t r i n g ( J o b G u i d )   T h e n  
                 b F o u n d   =   t r u e  
         E n d   I f  
          
         L o o k u p J o b   =   b F o u n d  
          
 e n d   f u n c t i o n  
  
 f u n c t i o n   L o o k u p T a g ( T a g ,   B y R e f   S i t e I D ,   B y R e f   C o m p a n y ,   B y R e f   R e l a y G U I D ,   B y R e f   D e v i c e G U I D ,   B y R e f   R e g i o n G U I D )  
         b F o u n d   =   f a l s e  
         '   D o   w e   n e e d   t o   c h e c k   r e g i o n   h e r e ?  
         S q l   =   " S E L E C T   T O P   1   R e l a y G U I D ,   O r g 1   a s   C o m p a n y ,   O r g 2   a s   S i t e A d d r ,   D e v i c e G U I D ,   R e g i o n G U I D   F R O M   R e l a y   W H E R E   b I s D e l   =   0   A N D   O r g 4   =   ' "   &   T a g   &   " '   O r d e r   b y   M o d D a t e   d e s c "  
         c a l l   a L o g ( " S Q L   =   "   &   S q l )  
         c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S q l ,   0 )  
  
         R e l a y G u i d   =   G e t V a r ( " d b _ R e l a y G U I D " )  
         I f   N o t   I s B l a n k S t r i n g ( R e l a y G u i d )   T h e n  
                 b F o u n d   =   t r u e  
                 C o m p a n y   =   G e t V a r ( " d b _ C o m p a n y " )  
                 s t r l e n   =   I n s t r ( G e t V a r ( " d b _ S i t e A d d r " ) , " - " )  
                 i f   ( s t r l e n   =   0 )   t h e n    
                         S i t e I D   =   G e t V a r ( " d b _ S i t e A d d r " )  
                 e l s e  
                         S i t e I D   =   L e f t ( G e t V a r ( " d b _ S i t e A d d r " ) , s t r l e n   -   1 )  
                 e n d   i f  
                 D e v i c e G U I D   =   G e t V a r ( " d b _ D e v i c e G U I D " )  
                 R e g i o n G U I D   =   G e t V a r ( " d b _ R e g i o n G U I D " )  
                 c a l l   S e t V a r ( " R e g i o n G U I D " , R e g i o n G U I D )  
         E n d   I f  
          
         L o o k u p T a g   =   b F o u n d  
 e n d   f u n c t i o n  
  
 S u b   L o o k u p S i t e ( )  
         V a l i d   =   V a l i d a t e S i t e F i e l d s ( T r u e )  
          
         I f   V a l i d   T h e n  
                 ' L o o k u p   S i t e  
                 S i t e I D   =   G e t V a r ( " S i t e I D " )  
                 S i t e I D   =   T r i m ( S i t e I D )  
                 c a l l   S e t V a r ( " S i t e I D " ,   S i t e I D )  
                 '   D o   w e   n e e d   t o   c h e c k   r e g i o n   h e r e ?  
                  
                 ' S q l   =   " S E L E C T   R e l a y G u i d ,   O r g 1   a s   C o m p a n y   F R O M   R e l a y   W H E R E   b I s D e l   =   0   A N D   O r g 2   =   ' "   &   S i t e I D   &   " '   "  
                 ' c a l l   a L o g ( " S Q L   =   "   &   S q l )  
                 ' c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S q l ,   0 )  
                  
                 C o m p a n y G u i d   =   L o a d S i t e A d d r e s s ( S i t e I D )  
  
                 ' R e l a y G u i d   =   G e t V a r ( " d b _ R e l a y G U I D " )  
                 I f   N o t   I s B l a n k S t r i n g ( C o m p a n y G u i d )   T h e n  
                         ' C o m p a n y   =   G e t V a r ( " d b _ C o m p a n y " )  
                         ' c a l l   S e t V a r ( " C o m p a n y " ,   C o m p a n y )  
                         ' c a l l   S e t V a r ( " R e l a y G u i d " ,   R e l a y G u i d )  
                         ' C o m p a n y G u i d   =   L o a d S i t e A d d r e s s ( S i t e I D )  
                         c a l l   S e t V a r (   " C o m p a n y G u i d " ,   C o m p a n y G u i d   )  
                          
                          
                         c a l l   S e t S c r e e n ( " C o n f i r m J o b C r e a t e " )  
                 E l s e  
                         c a l l   S e t S c r e e n ( " C o n f i r m S i t e C r e a t e " )  
                 E n d   I f  
          
         E n d   I f  
          
 E n d   S u b  
  
 S u b   L o o k u p S i t e B y C u s t o m e r ( )  
         S i t e I D   =   G e t V a r ( " S i t e I D " )  
         c a l l   S e t V a r ( " P r e F i n d S i t e I D " ,   S i t e I D )  
         c a l l   S e t V a r ( " S i t e I D " ,   " " )  
         c a l l   C l e a r S e a r c h F i e l d s ( )  
         c a l l   S e t S c r e e n ( " L o o k u p S i t e B y C u s t o m e r " )  
 E n d   S u b  
  
 S u b   U s e S i t e ( )  
         V a l i d   =   V a l i d a t e S i t e F i e l d s ( T r u e )  
          
         I f   V a l i d   T h e n  
                 S i t e I D   =   G e t V a r ( " S i t e I D " )  
                 S i t e I D   =   T r i m ( S i t e I D )  
                 c a l l   S e t V a r ( " S i t e I D " ,   S i t e I D )  
                 C o m p a n y G u i d   =   L o a d S i t e A d d r e s s ( S i t e I D )  
                  
                 c a l l   S e t V a r (   " C o m p a n y G u i d " ,   C o m p a n y G u i d   )  
                 c a l l   S e t S c r e e n ( " C o n f i r m J o b C r e a t e " )  
         E n d   I f  
 E n d   S u b  
  
 F u n c t i o n   C r e a t e J o b ( )  
         J o b N u m b e r   =   G e t V a r ( " T i c k e t " )   &   " - "   &   G e t V a r ( " T a g I D " )  
         C o m p a n y G u i d   =   G e t V a r ( " C o m p a n y G u i d " )  
         c a l l   a l o g ( " C O M P A N Y G U I D   "   &   C o m p a n y G u i d   )  
         i f   I s B l a n k S t r i n g ( C o m p a n y G u i d )   t h e n  
                 J o b G u i d   =   A p p l i c a t i o n . R u n A p p C m d ( - 1 ,   " C r e a t e J o b " ,   J o b N u m b e r   ,   0 )  
         e l s e  
                 J o b G u i d   =   A p p l i c a t i o n . R u n A p p C m d ( - 1 ,   " C r e a t e J o b " ,   J o b N u m b e r   &   " , "   &   C o m p a n y G u i d ,   0 )  
         e n d   i f  
  
         ' S q l   =   " S E L E C T   T o p   1   R e g i o n G U I D   F r o m   P d b R e g i o n s "  
         ' c a l l   a L o g ( " S Q L   =   "   &   S q l )  
         ' c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S q l ,   0 )  
          
         R e g i o n G U I D   =   G e t V a r ( " R e g i o n G U I D " )  
          
         i f   ( I s E m p t y ( R e g i o n G U I D )   o r   I s N u l l ( R e g i o n G U I D ) )   t h e n  
                 R e g i o n G U I D   =   G e t V a r ( " R e g i o n G U I D L i s t " )  
         e n d   i f  
            
          
         c a l l   a l o g ( " R E G I O N G U I D   "   &   R e g i o n G U I D   )  
          
         A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   R e g i o n G U I D   =   ' "   &   R e g i o n G U I D   &   " '   w h e r e   J o b G U I D   =   ' "   &   J o b G U I D   &   " ' " )  
         R e p o r t T y p e   =   G e t V a r ( " R e p o r t T y p e " )  
         s e l e c t   c a s e   R e p o r t T y p e  
         c a s e   0  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' B a t t e r y   M a i n t e n a n c e   R e p o r t '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )  
         c a s e   1  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' S i t e   S c a n   R e p o r t '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )  
         c a s e   2  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' A P M   R e p o r t '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i   &   " ' " )  
         c a s e   3  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' A P S   R e p o r t '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )  
         c a s e   4  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' N X   2 2 5 - 6 0 0   R e p o r t '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )  
         c a s e   5  
          
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' 1 0 7 7 1   R e p o r t '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )  
         c a s e   6  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' T H E R M A L   P M V '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )          
         c a s e   7  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' 8 0 8 0 0   U P S '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )    
         c a s e   8  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' E H S   S i t e   S u r v e y '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )    
         e n d   s e l e c t  
          
         C r e a t e J o b   =   J o b G u i d  
 E n d   F u n c t i o n  
  
 S u b   A d d A s s e t T o J o b ( J o b G u i d ,   R e l a y G u i d )  
  
         c a l l   a L o g ( " A d d A s s e t T o J o b :   J o b G u i d = "   &   J o b G u i d   &   " , R e l a y G u i d = "   &   R e l a y G u i d )  
  
         I f   N o t   I s B l a n k S t r i n g ( J o b G u i d )   A n d   N o t   I s B l a n k S t r i n g ( R e l a y G u i d )   T h e n          
                 c a l l   A p p l i c a t i o n . O p e n J o b ( J o b G u i d )      
                 R e t V a l   =   A p p l i c a t i o n . R u n A p p C m d ( - 1 ,   " A d d A s s e t T o J o b " ,   R e l a y G u i d   &   " , "   &   J o b G u i d   &   " , 1 " ,   0 )                  
         E n d   I f  
  
 E n d   S u b  
  
 S u b   C r e a t e J o b A d d A s s e t ( )  
  
         I f   V a l i d a t e C u s t o m e r F i e l d   T h e n  
                 J o b G u i d   =   C r e a t e J o b ( )  
                 R e l a y G u i d   =   G e t V a r ( " R e l a y G u i d " )  
                  
                 I f   I s B l a n k S t r i n g ( R e l a y G u i d )   T h e n  
                         c a l l   A p p l i c a t i o n . O p e n J o b ( J o b G u i d )      
                         c a l l   A d d N e w A s s e t T o J o b ( )  
                 E l s e  
                         c a l l   A d d A s s e t T o J o b ( J o b G u i d ,   R e l a y G u i d )  
                 E n d   I f          
                  
                 c a l l   E x i t P o p u p ( )  
                                  
                 F o r m . R e f r e s h   =   1  
         E n d   I f  
          
 E n d   S u b  
  
 S u b   C r e a t e J o b C r e a t e A s s e t ( )  
          
         I f   V a l i d a t e C u s t o m e r F i e l d   T h e n  
                 '   A d d   s i t e   a d d r e s s .    
                 c a l l   A p p l i c a t i o n . R u n A p p C m d ( F o r m . F o r m I d ,   " A d d P l a n t A d d r e s s " ,   " C u s t o m e r , A d d r e s s , A d d r e s s 2 , C i t y , S t a t e , C o u n t r y , Z i p , S i t e I D , F i r s t N a m e , L a s t N a m e " ,   1 )  
                  
                 C o m p a n y G u i d   =   L o a d S i t e A d d r e s s ( G e t V a r ( " S i t e I D " ) )  
                  
                 i f   N O T   I s B l a n k S t r i n g ( C o m p a n y G u i d )   t h e n  
                         c a l l   S e t V a r ( " C o m p a n y G u i d " ,   C o m p a n y G u i d )  
                 e n d   i f  
                          
                 J o b G u i d   =   C r e a t e J o b ( )  
                 c a l l   A p p l i c a t i o n . O p e n J o b ( J o b G u i d )              
                  
                 c a l l   A d d N e w A s s e t T o J o b ( )  
                  
                 c a l l   E x i t P o p u p ( )  
                  
                 F o r m . R e f r e s h   =   1  
         E n d   I f  
          
 E n d   S u b  
  
 S u b   A d d N e w A s s e t T o J o b ( )  
         O r g 1   =   G e t V a r ( " C u s t o m e r " )  
         O r g 2   =   G e t V a r ( " S i t e I D " )   &   " - "   &   G e t V a r ( " A d d r e s s " )  
         O r g 3   =   " < N o n e > "  
         O r g 4   =   G e t V a r ( " T a g I D " )  
          
         I f   O r g 1   =   " "   T h e n   O r g 1   =   " < N o n e > "  
          
         c a l l   a L o g ( " O r g 1   =   "   &   O r g 1 )  
         c a l l   a L o g ( " O r g 2   =   "   &   O r g 2 )  
         c a l l   a L o g ( " O r g 3   =   "   &   O r g 3 )  
         c a l l   a L o g ( " O r g 4   =   "   &   O r g 4 )  
          
         R e p o r t T y p e   =   G e t V a r ( " R e p o r t T y p e " )  
          
         s e l e c t   c a s e   R e p o r t T y p e  
                 c a s e   0  
                         F o r m G U I D   =   " B X I R A A A I 3 N H J 4 U S Q Y I S N "  
                 c a s e   1  
                         F o r m G U I D   =   " A U Q 3 A A A I R S U 2 J A S T 4 X S P "  
                 c a s e   2  
                         F o r m G U I D   =   " A V Y 3 A A A C Z B F P 2 7 P U 6 F 2 S "  
                 c a s e   3  
                         F o r m G U I D   =   " A V Y 3 A A A N D G K P J Q Y F C X S T "  
                 c a s e   4  
                         F o r m G U I D   =   " A V Y 3 A A A A D G 4 L K R G I E F C T "  
                 c a s e   5  
                         F o r m G U I D   =   " A V Y 3 A A A D S V 7 S H 2 M B M O C S "  
                 c a s e   6  
                         F o r m G U I D   =   " A 2 Y W A A A G N Z 2 B I S 6 P 7 U C U "  
                 c a s e   7  
                         F o r m G U I D   =   " A 2 Y W A A A O M T P Y L 2 I H Q J S W "  
                 c a s e   8  
                         F o r m G U I D   =   " A V Y 3 A A A O J T 2 J Q 6 B C E B S X "                  
                          
         e n d   s e l e c t  
                          
                          
         c a l l   A p p l i c a t i o n . A d d N e w F o r m ( O r g 1 ,   O r g 2 ,   O r g 3 ,   O r g 4 ,   F o r m G u i d )  
          
 E n d   S u b  
  
  
 S u b   C l e a r A d d r e s s F i e l d s ( )  
         C a l l   S e t V a r ( " C u s t o m e r " ,   " " )  
         C a l l   S e t V a r ( " A d d r e s s " ,   " " )  
         C a l l   S e t V a r ( " A d d r e s s 2 " ,   " " )  
         C a l l   S e t V a r ( " C i t y " ,   " " )  
         C a l l   S e t V a r ( " Z i p " ,   " " )  
         C a l l   S e t V a r ( " S t a t e " ,   " " )  
         C a l l   S e t V a r ( " C o u n t r y " ,   " " )          
 E n d   S u b  
  
 S u b   E x i t P o p u p ( )  
         F o r m . F o r m D l g R e t u r n V a l u e   =   " O K "  
         F o r m . F o r m D l g R e t u r n   =   " O K "  
 E n d   S u b  
  
 S u b   L o a d T a g F r o m F i l e ( )  
          
          
          
         S e t   o b j F S O   =   C r e a t e O b j e c t ( " S c r i p t i n g . F i l e S y s t e m O b j e c t " )  
         S e t   o b j F i l e   =   o b j F S O . O p e n T e x t F i l e ( " C : \ m o b i l e \ P D B _ I n s p e c t i o n . t x t " ,   1 )  
         s t r C o n t e n t s   =   o b j F i l e . R e a d A l l  
        
         o b j F i l e . C l o s e  
         S e t   o b j F i l e   =   N o t h i n g  
          
          
          
 E n d   S u b  
  
 F U N C T I O N   L o a d S i t e A d d r e s s ( S i t e I D )  
         S Q L   =   " S E L E C T   T O P   1   C . [ N a m e ] ,   C . C o m p a n y G U I D ,   A . A d d r G u i d ,   A . A d d r L n 1 ,   A . A d d r L n 2 ,   A . C i t y ,   A . S t a t e ,   A . C o u n t r y ,   A . Z i p ,   A . F i r s t N a m e ,   A . L a s t N a m e   "   & _  
                     "   F R O M   P d b A d d r I n f o   A   L E F T   J O I N   P d b A d d r H e a d e r   C   o n   A . C o m p a n y G u i d   =   C . C o m p a n y G U I D   "   & _  
                     "   W H E R E   A . [ T y p e ]   =   2   A n d   A . [ b I s D e l ]   =   0   A n d   ( A . A d d r N e u t r a l I d   =   ' "   &   S i t e I D   &   " '   O R   A . A d d r N e u t r a l I d   =   ' "   &   S i t e I D   &   " - 2 ' ) "  
          
         C a l l   S e t V a r ( " d b _ A d d r G u i d " ,   E m p t y )              
         c a l l   a L o g ( " S Q L = "   &   S Q L )  
          
         C a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . f o r m I d ,   S Q L ,   0 )  
          
         A d d r G u i d   =   G e t V a r ( " d b _ A d d r G u i d " )  
         I f   N o t   I s B l a n k S t r i n g ( A d d r G u i d )   T h e n  
                 C o m p a n y   =   G e t V a r ( " d b _ N a m e " )  
                 C o m p a n y G u i d   =   G e t V a r ( " d b _ C o m p a n y G u i d " )  
                 A d d r L n 1   =   G e t V a r ( " d b _ A d d r L n 1 " )  
                 A d d r L n 2   =   G e t V a r ( " d b _ A d d r L n 2 " )  
                 C i t y   =   G e t V a r ( " d b _ C i t y " )  
                 S t a t e   =   G e t V a r ( " d b _ S t a t e " )  
                 C o u n t r y   =   G e t V a r ( " d b _ C o u n t r y " )  
                 Z i p   =   G e t V a r ( " d b _ Z i p " )  
                 F i r s t N a m e   =   G e t V a r ( " d b _ F i r s t N a m e " )  
                 L a s t N a m e   =   G e t V a r ( " d b _ L a s t N a m e " )  
                  
                 C a l l   S e t V a r ( " C u s t o m e r " ,   C o m p a n y )  
                 C a l l   S e t V a r ( " C o m p a n y " ,   C o m p a n y )  
                 C a l l   S e t V a r ( " A d d r e s s " ,   A d d r L n 1 )  
                 C a l l   S e t V a r ( " A d d r e s s 2 " ,   A d d r L n 2 )  
                 C a l l   S e t V a r ( " C i t y " ,   C i t y )  
                 C a l l   S e t V a r ( " S t a t e " ,   S t a t e )  
                 C a l l   S e t V a r ( " t x t _ G e t S t a t e " ,   S t a t e )           '  
                 C a l l   S e t V a r ( " Z i p " ,   Z i p )  
                 C a l l   S e t V a r ( " C o u n t r y " ,   C o u n t r y )  
                 c a l l   S e t V a r ( " F i r s t N a m e A t t n " ,   F i r s t N a m e )  
                 c a l l   S e t V a r ( " L a s t N a m e A t t n " ,   L a s t N a m e )  
                 c a l l   S e t V a r ( " C u s t o m e r A t t n " ,   F i r s t N a m e   &   "   "   &   L a s t N a m e )  
                  
                 L o a d S i t e A d d r e s s   =   C o m p a n y G u i d  
                 c a l l   a L o g ( " F o u n d   S i t e   A d d r e s s :   "   &   A d d r G u i d   &   " ,   "   &   C o m p a n y G u i d   &   " ,   "   &   C o m p a n y   &   " ,   "   &   A d d r L n 1   &   " ,   "   &   A d d r L n 2   &   " ,   "   &   C i t y   &   " ,   "   &   S t a t e   &   " ,   "   &   Z i p   &   " ,   "   &   F i r s t N a m e   &   " ,   "   &   L a s t N a m e )  
         E l s e  
                 c a l l   a L o g ( " D i d   n o t   f i n d   s i t e   a d d r e s s " )  
         E n d   I f          
          
          
 E n d   F U N C T I O N  
  
 S u b   C h a n g e F o r m i n T a g ( T a g )  
          
         r e s u l t   =   m s g b o x ( " Y o u   a r e   a t t e m p t i n g   t o   m o d i f y   t h e   e q u i p m e n t   t y p e   o f   t h i s   t a g . "   +   v b C r L f   +   v b C r L f   + " D o   y o u   w i s h   t o   c o n t i n u e ? " , v b Y e s N o + v b E x c l a m a t i o n + v b D e f a u l t B u t t o n 2 , " W a r n i n g " )  
          
         i f   r e s u l t   =   6   t h e n  
                  
                 R e p o r t T y p e   =   G e t V a r ( " R e p o r t T y p e " )  
          
                 s e l e c t   c a s e   R e p o r t T y p e  
                         c a s e   0  
                                 F o r m G U I D   =   " B X I R A A A I 3 N H J 4 U S Q Y I S N "  
                         c a s e   1  
                                 F o r m G U I D   =   " A U Q 3 A A A I R S U 2 J A S T 4 X S P "  
                         c a s e   2  
                                 F o r m G U I D   =   " A V Y 3 A A A C Z B F P 2 7 P U 6 F 2 S "  
                         c a s e   3  
                                 F o r m G U I D   =   " A V Y 3 A A A N D G K P J Q Y F C X S T "  
                         c a s e   4  
                                 F o r m G U I D   =   " A V Y 3 A A A A D G 4 L K R G I E F C T "  
                         c a s e   5  
                                 F o r m G U I D   =   " A V Y 3 A A A D S V 7 S H 2 M B M O C S "  
                          
                 e n d   s e l e c t  
                  
                 S q l   =   " S E L E C T   D e v i c e N a m e   F R O M   D e v i c e _ T y p e   W H E R E   b I s D e l   =   0   A N D   D e v i c e G U I D   =   ' "   &   F o r m G U I D   &   " '   "  
                 c a l l   a L o g ( " S Q L   =   "   &   S q l )  
                 c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S q l ,   0 )  
                 R e l a y N a m e   =   G e t V a r ( " d b _ D e v i c e N a m e " )  
  
                 c a l l   L o o k u p T a g ( T a g ,   S i t e I D ,   C o m p a n y ,   R e l a y G U I D ,   D e v i c e G U I D ,   R e g i o n G U I D )      
                 c a l l   S e t V a r ( " S i t e I D " ,   S i t e I D )  
                 c a l l   S e t V a r ( " T a g I D " ,   T a g )  
                 c a l l   S e t V a r ( " C u s t o m e r " ,   C o m p a n y )              
                 c a l l   A d d N e w A s s e t T o J o b ( )  
                 ' A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   R e l a y   S e t   D e v i c e G U I D   =   ' "   &   F o r m G U I D   &   " ' ,   R e l a y N a m e   =   ' "   &   R e l a y N a m e   &   " '   W H E R E   b I s D e l   =   0   A N D   O r g 3   =   ' "   &   T a g   &   " '   " )  
  
                 E r r M s g   =   " "  
                 c a l l   S e t V a r ( " R e p o r t T y p e E r r " ,   E r r M s g )  
                 c a l l   S e t V a r ( " U p d a t e T a g " ,   0 )  
  
                  
         e n d   i f  
          
          
 E n d   S u b  
  
     mE#џўџD a t a џўџ            РРР џўџ0                џўџ             mE#         $2         џўџ       РРР p  р  р  p                mE#џўџO n   I n i t i a l   U p d a t e џўџc a l l   N e w J o b I n i t ( )     mE#џўџ	O n   U p d a t e џўџAc a l l   N e w J o b O n U p d a t e ( )  
 c a l l   S e t V a r ( " S c r i p t T i m e o u t M s " ,   C I n t ( - 1 ) )  
      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџџЖkD i m   g _ U I S t a c k  
 S e t   g _ U I S t a c k   =   n e w   S t a c k  
  
 c l a s s   S t a c k  
        
         p r i v a t e   s u b   c l a s s _ I n i t i a l i z e ( )  
                 c a l l   a L o g ( " c l a s s _ I n i t i a l i z e " )  
                 P o s   =   G e t V a r ( " S t a c k P o s " )  
                  
                 I f   I s B l a n k S t r i n g ( P o s )   T h e n  
                         c a l l   S e t V a r ( " S t a c k P o s " ,   0 )  
                 E n d   I f  
                  
         e n d   s u b  
          
         p u b l i c   s u b   P u s h ( I t e m )          
                 P o s   =   G e t V a r ( " S t a c k P o s " )  
                  
                 P o s   =   P o s   +   1  
                 c a l l   S e t N d x V a r ( " S t a c k D a t a " ,   P o s ,   I t e m )  
                 c a l l   S e t V a r ( " S t a c k P o s " ,   P o s )  
                  
         e n d   s u b  
          
         p u b l i c   f u n c t i o n   P o p ( )          
                 I t e m   =   P e e k ( )  
                  
                 P o s   =   G e t V a r ( " S t a c k P o s " )  
                 I f   P o s   >   0   T h e n  
                         P o s   =   P o s   -   1  
                         c a l l   S e t V a r ( " S t a c k P o s " ,   P o s )  
                 E n d   I f  
                  
                 P o p   =   I t e m  
         e n d   f u n c t i o n  
          
         p u b l i c   f u n c t i o n   P e e k ( )  
                 P o s   =   G e t V a r ( " S t a c k P o s " )  
                  
                 I f   P o s   >   0   T h e n  
                         I t e m   =   G e t N d x V a r ( " S t a c k D a t a " ,   P o s )  
                 E l s e  
                         I t e m   =   " "  
                 E n d   I f  
                 P e e k   =   I t e m  
         e n d   f u n c t i o n  
          
         p u b l i c   s u b   P r i n t S t a c k ( )  
                 S t r   =   " "  
                  
                 P o s   =   G e t V a r ( " S t a c k P o s " )  
                 f o r   i   =   P o s   t o   1   S t e p   - 1  
                         S t r   =   S t r   &   G e t N d x V a r ( " S t a c k D a t a " ,   i )   &   C h r ( 1 3 )   &   C h r ( 1 0 )  
                 n e x t  
                 c a l l   a L o g ( " S t a c k = "   &   S t r )  
         e n d   s u b  
          
         p u b l i c   s u b   C l e a r ( )  
                 c a l l   S e t V a r ( " S t a c k P o s " ,   0 )  
         e n d   s u b  
          
 e n d   c l a s s  
  
 s u b   N e w J o b I n i t ( )                
          
         F i l e N a m e   = " C : \ m o b i l e \ P D B _ I n s p e c t i o n . t x t "  
         s t r N e w N a m e   =   " C : \ m o b i l e \ P D B _ I n s p e c t i o n . o l d "  
          
         S e t   o b j F S O   =   C r e a t e O b j e c t ( " S c r i p t i n g . F i l e S y s t e m O b j e c t " )  
          
         i f   ( o b j F S O . F i l e E x i s t s ( F i l e N a m e ) )   t h e n  
                  
                 S e t   o b j F i l e   =   o b j F S O . O p e n T e x t F i l e ( F i l e N a m e ,   1 )  
          
                 D o   U n t i l   o b j F i l e . A t E n d O f S t r e a m  
                         s t r L i n e   =   o b j F i l e . R e a d L i n e  
                         c a l l   a l o g ( " s t r L i n e :   "   +   s t r L i n e )  
                         i f   (   L e f t ( s t r L i n e , 2 )   < >   " / / "   )   t h e n  
                                 i f   ( I n S t r ( s t r L i n e , " T a g = " ) )   t h e n  
                                         T a g I D   =   M i d ( s t r L i n e , 5 )  
                                         c a l l   S e t V a r ( " T a g I D " , T a g I D )  
                                 e l s e i f   ( I n S t r ( s t r L i n e , " T i c k e t = " ) )   t h e n  
                                         T i c k e t   =   M i d ( s t r L i n e , 8 )  
                                         c a l l   S e t V a r ( " T i c k e t " , T i c k e t )  
                                 e l s e i f   ( I n S t r ( s t r L i n e , " R e p o r t = " ) )   t h e n  
                                         R e p o r t T y p e   =   M i d ( s t r L i n e , 8 )  
                                         s e l e c t   c a s e   R e p o r t T y p e  
                                         c a s e   " 1 0 7 7 0 "  
                                                 F o r m G U I D   =   " B X I R A A A I 3 N H J 4 U S Q Y I S N "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 0 )  
                                         c a s e   " 2 0 1 0 0 "  
                                                 F o r m G U I D   =   " A U Q 3 A A A I R S U 2 J A S T 4 X S P "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 1 )  
                                         c a s e   " 8 0 2 0 0 "  
                                                 F o r m G U I D   =   " A V Y 3 A A A C Z B F P 2 7 P U 6 F 2 S "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 2 )  
                                         c a s e   " 8 0 1 0 0 "  
                                                 F o r m G U I D   =   " A V Y 3 A A A N D G K P J Q Y F C X S T "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 3 )  
                                         c a s e   " 8 0 3 0 0 "  
                                                 F o r m G U I D   =   " A V Y 3 A A A A D G 4 L K R G I E F C T "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 4 )  
                                         c a s e   " 1 0 7 7 1 "  
                                                 F o r m G U I D   =   " A V Y 3 A A A D S V 7 S H 2 M B M O C S "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 5 )  
                                         c a s e   " T H E R M A L   P M V "  
                                                 F o r m G U I D   =   " A 2 Y W A A A G N Z 2 B I S 6 P 7 U C U "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 6 )  
                                         c a s e   " 8 0 8 0 0   U P S "  
                                                 F o r m G U I D   =   " A 2 Y W A A A G N Z 2 B I S 6 P 7 U C U "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 7 )  
                                         c a s e   " E H S   S i t e   S u r v e y "  
                                                 F o r m G U I D   =   " A V Y 3 A A A O J T 2 J Q 6 B C E B S X "  
                                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 8 )  
  
                                         e n d   s e l e c t  
                                 e n d   i f  
                         e n d   i f  
                 L o o p  
                        
                 o b j F i l e . C l o s e  
                 S e t   o b j F i l e   =   N o t h i n g  
                  
                 i f   ( L o o k u p T a g ( T a g I D ,   S i t e I D ,   C o m p a n y ,   R e l a y G U I D ,   D e v i c e G U I D ,   R e g i o n G U I D ) )   A n d   ( F o r m G U I D   =   D e v i c e G U I D )   T h e n  
                         c a l l   S e t V a r ( " T a g I D " , T a g I D )  
                         c a l l   a l o g ( " T a g I D =   "   +   T a g I D )  
                         c a l l   S e t V a r ( " T i c k e t " , T i c k e t )  
                         c a l l   a L o g ( " R e l a y G U I D = "   &   R e l a y G U I D   &   " , C o m p a n y = "   &   C o m p a n y   &   " , S i t e I D = "   &   S i t e I D   &   " , F o r m I D = "   &   D e v i c e G U I D )  
                         s e l e c t   c a s e   R e p o r t T y p e  
                         c a s e   " 1 0 7 7 0 "  
                                 F o r m G U I D   =   " B X I R A A A I 3 N H J 4 U S Q Y I S N "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 0 )  
                         c a s e   " 2 0 1 0 0 "  
                                 F o r m G U I D   =   " A U Q 3 A A A I R S U 2 J A S T 4 X S P "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 1 )  
                         c a s e   " 8 0 2 0 0 "  
                                 F o r m G U I D   =   " A V Y 3 A A A C Z B F P 2 7 P U 6 F 2 S "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 2 )  
                         c a s e   " 8 0 1 0 0 "  
                                 F o r m G U I D   =   " A V Y 3 A A A N D G K P J Q Y F C X S T "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 3 )  
                         c a s e   " 8 0 3 0 0 "  
                                 F o r m G U I D   =   " A V Y 3 A A A A D G 4 L K R G I E F C T "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 4 )  
                         c a s e   " 1 0 7 7 1 "  
                                 F o r m G U I D   =   " A V Y 3 A A A D S V 7 S H 2 M B M O C S "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 5 )  
                         c a s e   " T H E R M A L   P M V "  
                                 F o r m G U I D   =   " A 2 Y W A A A G N Z 2 B I S 6 P 7 U C U "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 6 )  
                         c a s e   " U P S "  
                                 F o r m G U I D   =   " A 2 Y W A A A O M T P Y L 2 I H Q J S W "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 7 )  
                         c a s e   " E H S   S i t e   S u r v e y "  
                                 F o r m G U I D   =   " A V Y 3 A A A O J T 2 J Q 6 B C E B S X "  
                                 c a l l   S e t V a r ( " R e p o r t T y p e " , 8 )  
  
                         e n d   s e l e c t  
                          
                         i f   o b j F S O . F i l e E x i s t s ( s t r N e w N a m e )   t h e n  
                                 o b j F S O . D e l e t e F i l e   s t r N e w N a m e  
                         e n d   i f  
                          
                         o b j F S O . M o v e F i l e   F i l e N a m e ,   s t r N e w N a m e  
                 e l s e  
                         c a l l   a l o g ( " R e p o r t   T y p e   f r o m   f i l e :   "   &   F o r m G U I D )  
                         c a l l   a l o g ( " R e p o r t   T y p e   f r o m   T a g :   "   &   D e v i c e G U I D )  
                         c a l l   S e t V a r ( " R e p o r t T y p e " , " " )  
                         c a l l   a l o g ( " T a g   n o t   f o u n d   o r   i n c o r r e c t   r e p o r t   t y p e   i n   P D B _ I n s p e c t i o n . t x t :   D e l e t i n g   F i l e " )  
                         o b j F S O . D e l e t e F i l e   F i l e n a m e  
                 e n d   i f  
         e n d   i f  
         S e t S c r e e n ( " L o o k u p " )  
          
 e n d   s u b  
  
 s u b   S e t S c r e e n ( N e w S c r e e n )        
         c a l l   a L o g ( " S e t S c r e e n ( "   &   N e w S c r e e n   &   " )   S T A R T   - - - - - - - - - - - " )  
         c a l l   S e t V a r ( " U I S t a t e " ,   N e w S c r e e n )  
         c a l l   g _ U I S t a c k . P u s h ( N e w S c r e e n )  
         c a l l   g _ U I S t a c k . P r i n t S t a c k ( )  
         c a l l   a L o g ( " S e t S c r e e n ( "   &   N e w S c r e e n   &   " )   E N D   - - - - - - - - - - - " )  
 e n d   s u b  
  
 s u b   G o B a c k ( )  
         c a l l   a L o g ( " G o B a c k ( "   &   N e w S c r e e n   &   " )   S T A R T   - - - - - - - - - - - " )  
         g _ U I S t a c k . P o p ( )  
         S c r e e n   =   g _ U I S t a c k . P e e k ( )  
          
         c a l l   S e t V a r ( " U I S t a t e " ,   S c r e e n )                  
         c a l l   a L o g ( " G o B a c k ( "   &   N e w S c r e e n   &   " )   E N D   - - - - - - - - - - - " )  
 e n d   s u b  
  
  
 s u b   N e w J o b O n U p d a t e ( )  
         U I S t a t e   =   G e t V a r ( " U I S t a t e " )  
         c a l l   a L o g ( " N e w J o b O n U p d a t e :   U I S t a t e   =   "   &   U I S t a t e )  
          
         S u b F o r m s   =   A r r a y ( " L o o k u p " ,   " C o n f i r m J o b C r e a t e " ,   " C o n f i r m T a g C r e a t e " ,   " L o o k u p S i t e " ,   " L o o k u p S i t e B y C u s t o m e r " ,   " C o n f i r m S i t e C r e a t e " ,   " C r e a t e N e w " )  
         H i d d e n T a g s   =   " "  
          
         f o r   i   =   0   t o   U B o u n d ( S u b F o r m s )  
                 S u b f o r m S u f f i x   =   S u b F o r m s ( i )  
                 S u b f o r m N a m e   =   " _ _ N e w J o b S u b f o r m "   &   S u b f o r m S u f f i x  
                  
                 i f   S u b f o r m S u f f i x   =   U I S t a t e   t h e n  
                         c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   S u b f o r m N a m e ,   " V i s i b l e   1 : S e n d T o F r o n t   1 " )  
                         c a l l   S e t V a r ( " M o d a l S u b f o r m " ,   S u b f o r m N a m e )  
                 e l s e  
                         H i d d e n T a g s   =   H i d d e n T a g s   &   S u b f o r m N a m e   &   " , "  
                 e n d   i f                  
         n e x t  
         c a l l   a L o g ( " H i d d e n T a g s   =   "   &   H i d d e n T a g s )  
         c a l l   S e t V i s i b l e ( H i d d e n T a g s ,   0 )  
          
          
          
          
         s e l e c t   c a s e   U I S t a t e  
                 c a s e   " L o o k u p "  
                         c a l l   V a l i d a t e L o o k u p F i e l d s ( F a l s e )  
                 c a s e   " C o n f i r m J o b C r e a t e "  
                 c a s e   " C o n f i r m T a g C r e a t e "  
                 c a s e   " L o o k u p S i t e "  
                         c a l l   V a l i d a t e S i t e F i e l d s ( F a l s e )                  
                 c a s e   " L o o k u p S i t e B y C u s t o m e r "  
                 c a s e   " C o n f i r m S i t e C r e a t e "  
                 c a s e   " C o n f i r m T a g C r e a t e "  
         e n d   s e l e c t  
          
 e n d   s u b  
  
 f u n c t i o n   V a l i d a t e L o o k u p F i e l d s ( V a l i d a t e R e q u i r e d )  
         b V a l i d   =   t r u e  
          
         b V a l i d   =   b V a l i d   A N D   V a l i d a t e N u m e r i c F i e l d ( " T i c k e t " ,   7 ,   V a l i d a t e R e q u i r e d )  
         b V a l i d   =   b V a l i d   A N D   V a l i d a t e N u m e r i c F i e l d ( " T a g I D " ,   7 ,   V a l i d a t e R e q u i r e d )  
                  
         V a l i d a t e L o o k u p F i e l d s   =   b V a l i d          
 e n d   f u n c t i o n  
  
 f u n c t i o n   V a l i d a t e L o o k u p F i e l d s T a g I D ( V a l i d a t e R e q u i r e d )  
     '     b V a l i d   =   t r u e  
          
     '     b V a l i d   =   b V a l i d   A N D   V a l i d a t e N u m e r i c F i e l d R a n g e ( " T a g I D " ,   4 , 7 ,   V a l i d a t e R e q u i r e d )  
                  
       '   V a l i d a t e L o o k u p F i e l d s   =   b V a l i d          
 e n d   f u n c t i o n  
  
 f u n c t i o n   V a l i d a t e S i t e F i e l d s ( V a l i d a t e R e q u i r e d )  
         b V a l i d   =   t r u e  
          
         b V a l i d   =   b V a l i d   A N D   V a l i d a t e N u m e r i c F i e l d ( " S i t e I D " ,   6 ,   V a l i d a t e R e q u i r e d )  
          
         V a l i d a t e S i t e F i e l d s   =   b V a l i d          
 e n d   f u n c t i o n  
  
 f u n c t i o n   V a l i d a t e C u s t o m e r F i e l d ( )  
         b V a l i d   =   t r u e  
          
         b V a l i d   =   b V a l i d   A N D   V a l i d a t e I s N o t B l a n k F i e l d ( " C u s t o m e r " )  
          
         V a l i d a t e C u s t o m e r F i e l d   =   b V a l i d          
 e n d   f u n c t i o n  
  
 f u n c t i o n   V a l i d a t e R e p o r t T y p e F i e l d ( )  
         b V a l i d   =   t r u e  
          
         b V a l i d   =   b V a l i d   A N D   V a l i d a t e I s N o t B l a n k F i e l d ( " R e p o r t T y p e " )  
          
         V a l i d a t e R e p o r t T y p e F i e l d   =   b V a l i d          
 e n d   f u n c t i o n  
  
 f u n c t i o n   V a l i d a t e I s N o t B l a n k F i e l d ( F i e l d N a m e )  
                  
         V a l i d   =   t r u e  
          
         V a l u e   =   G e t V a r ( F i e l d N a m e )  
          
         V a l u e   =   T r i m ( V a l u e )  
         V a l i d   =   V a l i d   A N D   N O T   I s B l a n k S t r i n g ( V a l u e )   A N D   V a l u e   < >   " - 1 "  
          
         I f   V a l i d   T h e n  
                 c a l l   S e t V a r ( F i e l d N a m e   &   " E r r " ,   " " )  
         E l s e  
               '   E r r M s g   =   F i e l d N a m e   &   "   i s   r e q u i r e d "            
                 E r r M s g   =   " T h i s   f i e l d   i s   r e q u i r e d "           ' K i r a  
                 c a l l   S e t V a r ( F i e l d N a m e   &   " E r r " ,   E r r M s g )  
         E n d   I f  
  
         V a l i d a t e I s N o t B l a n k F i e l d   =   V a l i d  
          
 e n d   f u n c t i o n  
  
 f u n c t i o n   V a l i d a t e N u m e r i c F i e l d ( F i e l d N a m e ,   L e n g t h ,   V a l i d a t e R e q u i r e d )  
          
         D i m   R e g E x  
                  
         V a l i d   =   t r u e  
          
         V a l u e   =   G e t V a r ( F i e l d N a m e )  
          
         V a l u e   =   T r i m ( V a l u e )  
         V a l L e n g t h   =   L e n ( V a l u e )  
         E r r M s g   =   " "  
          
         c a l l   a L o g ( " V a l i d a t e N u m e r i c F i e l d : "   &   F i e l d N a m e   &   " , v a l u e = "   &   V a l u e   &   " , L e n g t h = "   &   V a l L e n g t h   &   " , r e q = "   &   V a l i d a t e R e q u i r e d )  
          
         I f   V a l L e n g t h   =   0   T h e n  
          
                 '   R e q u i r e d  
                 c a l l   a L o g ( " B e f o r e   V a l i d a t e R e q u i r e d   : "   &   V a l i d a t e R e q u i r e d )  
                 c a l l   a L o g ( " V a l i d a t e   R e q u i r e d " )  
                 V a l i d   =   F a l s e  
                  
                 R e q u i r e d E r r M s g   =   F i e l d N a m e   &   "   i s   r e q u i r e d "  
                  
                 I f   V a l i d a t e R e q u i r e d   O r   G e t V a r ( F i e l d N a m e   &   " E r r " )   =   R e q u i r e d E r r M s g   T h e n                          
                         E r r M s g   =   R e q u i r e d E r r M s g  
                 E n d   I f  
         E l s e  
                 S e t   R e g E x   =   N e w   R e g E x p  
                 R e g E x . P a t t e r n   =   " ^ \ d { "   &   L e n g t h   &   " } $ "  
                 V a l i d   =   R e g E x . T e s t ( V a l u e )  
                 I f   N o t   V a l i d   T h e n  
                         ' E r r M s g   =   F i e l d N a m e   &   "   s h o u l d   b e   n u m e r i c   a n d   "   &   L e n g t h   &   "   d i g i t s   l o n g . "  
                         ' E r r M s g   =   " T h i s   f i e l d   s h o u l d   b e   n u m e r i c   a n d   "   &   L e n g t h   &   "   d i g i t s   l o n g . "  
                         E r r M s g   =   " M U S T   b e   "   &   L e n g t h   &   "   d i g i t   n o . ,   a d d   l e a d i n g   z e r o   i f   r e q u i r e d . "  
                 E n d   I f                  
         E n d   I f  
          
         c a l l   a L o g ( " V a l i d = "   &   V a l i d   &   " ,   E r r M s g   = "   &   E r r M s g )  
         I f   V a l i d   T h e n  
                 c a l l   S e t V a r ( F i e l d N a m e   &   " E r r " ,   " " )  
         E l s e  
                 c a l l   S e t V a r ( F i e l d N a m e   &   " E r r " ,   E r r M s g )  
         E n d   I f  
  
         V a l i d a t e N u m e r i c F i e l d   =   V a l i d  
          
 e n d   f u n c t i o n    
  
 f u n c t i o n   V a l i d a t e N u m e r i c F i e l d R a n g e ( F i e l d N a m e ,   S t a r t ,   E n d V a l u e ,   V a l i d a t e R e q u i r e d )  
          
         D i m   R e g E x  
                  
         V a l i d   =   t r u e  
          
         V a l u e   =   G e t V a r ( F i e l d N a m e )  
          
         V a l u e   =   T r i m ( V a l u e )  
         V a l L e n g t h   =   L e n ( V a l u e )  
         E r r M s g   =   " "  
          
         c a l l   a L o g ( " V a l i d a t e N u m e r i c F i e l d : "   &   F i e l d N a m e   &   " , v a l u e = "   &   V a l u e   &   " , L e n g t h = "   &   V a l L e n g t h   &   " , r e q = "   &   V a l i d a t e R e q u i r e d )  
          
         I f   V a l L e n g t h   =   0   T h e n  
          
                 '   R e q u i r e d  
                 c a l l   a L o g ( " B e f o r e   V a l i d a t e R e q u i r e d   : "   &   V a l i d a t e R e q u i r e d )  
                 c a l l   a L o g ( " V a l i d a t e   R e q u i r e d " )  
                 V a l i d   =   F a l s e  
                  
                 ' R e q u i r e d E r r M s g   =   F i e l d N a m e   &   "   i s   r e q u i r e d "  
                 R e q u i r e d E r r M s g   =   " T h i s   f i e l d   i s   r e q u i r e d "  
                  
                 I f   V a l i d a t e R e q u i r e d   O r   G e t V a r ( F i e l d N a m e   &   " E r r " )   =   R e q u i r e d E r r M s g   T h e n                          
                         E r r M s g   =   R e q u i r e d E r r M s g  
                 E n d   I f  
         E l s e  
                 S e t   R e g E x   =   N e w   R e g E x p  
                 R e g E x . P a t t e r n   =   " ^ \ d { "   &   S t a r t   &   " , "   &   E n d V a l u e   &   " } $ "  
                 V a l i d   =   R e g E x . T e s t ( V a l u e )  
                 I f   N o t   V a l i d   T h e n  
                         ' E r r M s g   =   F i e l d N a m e   &   "   s h o u l d   b e   n u m e r i c   a n d   "   &   S t a r t   &   " - "   &   E n d V a l u e   &   "   d i g i t s . "  
                         E r r M s g   =   " T h i s   f i e l d   s h o u l d   b e   n u m e r i c   a n d   "   &   S t a r t   &   " - "   &   E n d V a l u e   &   "   d i g i t s . "  
                 E n d   I f                  
         E n d   I f  
          
         c a l l   a L o g ( " V a l i d = "   &   V a l i d   &   " ,   E r r M s g   = "   &   E r r M s g )  
         I f   V a l i d   T h e n  
                 c a l l   S e t V a r ( F i e l d N a m e   &   " E r r " ,   " " )  
         E l s e  
                 c a l l   S e t V a r ( F i e l d N a m e   &   " E r r " ,   E r r M s g )  
         E n d   I f  
  
         V a l i d a t e N u m e r i c F i e l d R a n g e   =   V a l i d  
          
 e n d   f u n c t i o n    
  
 s u b   L o o k u p T i c k e t A n d T a g ( )  
         D i m   J o b G u i d  
         D i m   S i t e I D  
         D i m   C o m p a n y  
         D i m   R e l a y G u i d  
          
         c a l l   a L o g ( " B e f o r e " )  
         c a l l   S e t V a r ( " R e l a y G U I D " ,   E m p t y )  
         c a l l   S e t V a r ( " C u s t o m e r " ,   E m p t y )  
         c a l l   S e t V a r ( " S i t e I D " ,   E m p t y )  
         c a l l   S e t V a r ( " D e v i c e G U I D " ,   E m p t y )          
  
         V a l i d   =   V a l i d a t e L o o k u p F i e l d s ( t r u e )  
         c a l l   a L o g ( " L o o k u p T i c k e t A n d T a g   = "   &   V a l i d )  
          
          
         I f   V a l i d   T h e n  
                 T i c k e t   =   G e t V a r ( " T i c k e t " )  
                 T a g I D   =   G e t V a r ( " T a g I D " )  
                 c a l l   S e t V a r ( " U p d a t e T a g " ,   0 )          
                 J o b N u m   =   T i c k e t   &   " - "   &   T a g I D  
                 I f   L o o k u p R e p o r t ( T a g I D ,   D e v i c e G U I D )   t h e n                  
                         I f   L o o k u p J o b ( J o b N u m ,   J o b G u i d )   T h e n  
                                 '   O p e n   J o b  
                                 c a l l   A p p l i c a t i o n . O p e n J o b ( J o b G u i d )  
                                 c a l l   E x i t P o p u p ( )  
                         E l s e I f   L o o k u p T a g ( T a g I D ,   S i t e I D ,   C o m p a n y ,   R e l a y G U I D ,   D e v i c e G U I D ,   R e g i o n G U I D )   T h e n  
                                 c a l l   a L o g ( " R e l a y G U I D = "   &   R e l a y G u i d   &   " , C o m p a n y = "   &   C o m p a n y   &   " , S i t e I D = "   &   S i t e I D )  
                                 c a l l   S e t V a r ( " R e l a y G U I D " ,   R e l a y G U I D )  
                                 c a l l   S e t V a r ( " C u s t o m e r " ,   C o m p a n y )  
                                 c a l l   S e t V a r ( " S i t e I D " ,   S i t e I D )  
                                 c a l l   S e t V a r ( " R e g i o n G U I D " , R e g i o n G U I D )  
                                  
                                 '   L o o k u p   s i t e   a d d r e s s  
                                 C o m p a n y G u i d   =   L o a d S i t e A d d r e s s ( S i t e I D )  
                                  
                                 i f   N O T   I s B l a n k S t r i n g ( C o m p a n y G u i d )   t h e n  
                                         c a l l   S e t V a r ( " C o m p a n y G u i d " ,   C o m p a n y G u i d )  
                                 e n d   i f  
                                  
                                 c a l l   S e t S c r e e n ( " C o n f i r m J o b C r e a t e " )  
                         E l s e  
                                 c a l l   S e t S c r e e n ( " C o n f i r m T a g C r e a t e " )  
                         E n d   I f  
                 E l s e          
                         E r r M s g   =   " I n v a l i d   r e p o r t   t y p e "  
                         c a l l   S e t V a r ( " R e p o r t T y p e E r r " ,   E r r M s g )  
                         c a l l   S e t V a r ( " U p d a t e T a g " ,   1 )  
                 E n d   I f  
         E n d   I f  
          
 e n d   s u b  
  
 f u n c t i o n   L o o k u p R e p o r t ( T a g ,   B y R e f   D e v i c e G U I D )  
          
         b F o u n d   =   f a l s e  
          
         S q l   =   " S E L E C T   T O P   1   D e v i c e G U I D   F R O M   R e l a y   W H E R E   b I s D e l   =   0   A N D   O r g 4   =   ' "   &   T a g   &   " '   O r d e r   b y   M o d D a t e   d e s c "  
         c a l l   a L o g   ( " S Q L   =   "   &   S q l )  
         c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S q l ,   0 )  
          
         D e v i c e G U I D   =   G e t V a r ( " d b _ D e v i c e G U I D " )  
  
         R e p o r t T y p e   =   G e t V a r ( " R e p o r t T y p e " )  
         c a l l   a L o g ( " R e p o r t   T y p e   =   "   &   R e p o r t T y p e )  
         s e l e c t   c a s e   R e p o r t T y p e  
                 c a s e   0  
                         F o r m G U I D   =   " B X I R A A A I 3 N H J 4 U S Q Y I S N "  
                 c a s e   1  
                         F o r m G U I D   =   " A U Q 3 A A A I R S U 2 J A S T 4 X S P "  
                 c a s e   2  
                         F o r m G U I D   =   " A V Y 3 A A A C Z B F P 2 7 P U 6 F 2 S "  
                 c a s e   3  
                         F o r m G U I D   =   " A V Y 3 A A A N D G K P J Q Y F C X S T "  
                 c a s e   4  
                         F o r m G U I D   =   " A V Y 3 A A A A D G 4 L K R G I E F C T "  
                 c a s e   5  
                         F o r m G U I D   =   " A V Y 3 A A A D S V 7 S H 2 M B M O C S "  
                 c a s e   6  
                         F o r m G U I D   =   " A 2 Y W A A A G N Z 2 B I S 6 P 7 U C U "  
                 c a s e   7  
                         F o r m G U I D   =   " A 2 Y W A A A O M T P Y L 2 I H Q J S W "  
                 c a s e   8  
                         F o r m G U I D   =   " A V Y 3 A A A O J T 2 J Q 6 B C E B S X "  
                          
         e n d   s e l e c t          
         c a l l   a L o g ( " D e v i c e G U I D   =   "   &   D e v i c e G U I D )  
         c a l l   a L o g ( " F o r m G U I D   =   "   &   F o r m G U I D )  
         i f   (   D e v i c e G U I D   =   F o r m G U I D   )   O R   (   D e v i c e G U I D   =   " "   ) t h e n  
                 b F o u n d   =   t r u e  
         e n d   i f  
          
         L o o k u p R e p o r t   =   b F o u n d  
          
 e n d   f u n c t i o n  
  
 f u n c t i o n   L o o k u p J o b ( J o b N u m ,   B y R e f   J o b G u i d )  
          
         b F o u n d   =   f a l s e  
          
         '   T o D o :   R e s t r i c t   t o   j o b s   i n   u s e r s   r e g i o n   ( i f   s e t   i n   u s e r   a c c o u n t ) .  
         '   R e s t r i c t   t o   o w n   j o b s   ( i f   s e t   i n   u s e r   a c c o u n t )  
         S q l   =   " S E L E C T   J o b G u i d   F R O M   P d b J o b   W H E R E   J o b N u m b e r   =   ' "   &   J o b N u m   &   " '   A N D   b I s D e l   =   0 "  
         c a l l   a L o g ( " S Q L   =   "   &   S q l )  
         c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S q l ,   0 )  
          
         J o b G u i d   =   G e t V a r ( " d b _ J o b G u i d " )  
         c a l l   a L o g ( " J o b G u i d = "   &   J o b G u i d )  
         c a l l   a L o g ( " J o b N u m b e r = "   &   J o b N u m )  
         I f   N o t   I s B l a n k S t r i n g ( J o b G u i d )   T h e n  
                 b F o u n d   =   t r u e  
         E n d   I f  
          
         L o o k u p J o b   =   b F o u n d  
          
 e n d   f u n c t i o n  
  
 f u n c t i o n   L o o k u p T a g ( T a g ,   B y R e f   S i t e I D ,   B y R e f   C o m p a n y ,   B y R e f   R e l a y G U I D ,   B y R e f   D e v i c e G U I D ,   B y R e f   R e g i o n G U I D )  
         b F o u n d   =   f a l s e  
         '   D o   w e   n e e d   t o   c h e c k   r e g i o n   h e r e ?  
         S q l   =   " S E L E C T   T O P   1   R e l a y G U I D ,   O r g 1   a s   C o m p a n y ,   O r g 2   a s   S i t e A d d r ,   D e v i c e G U I D ,   R e g i o n G U I D   F R O M   R e l a y   W H E R E   b I s D e l   =   0   A N D   O r g 4   =   ' "   &   T a g   &   " '   O r d e r   b y   M o d D a t e   d e s c "  
         c a l l   a L o g ( " S Q L   =   "   &   S q l )  
         c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S q l ,   0 )  
  
         R e l a y G u i d   =   G e t V a r ( " d b _ R e l a y G U I D " )  
         I f   N o t   I s B l a n k S t r i n g ( R e l a y G u i d )   T h e n  
                 b F o u n d   =   t r u e  
                 C o m p a n y   =   G e t V a r ( " d b _ C o m p a n y " )  
                 s t r l e n   =   I n s t r ( G e t V a r ( " d b _ S i t e A d d r " ) , " - " )  
                 i f   ( s t r l e n   =   0 )   t h e n    
                         S i t e I D   =   G e t V a r ( " d b _ S i t e A d d r " )  
                 e l s e  
                         S i t e I D   =   L e f t ( G e t V a r ( " d b _ S i t e A d d r " ) , s t r l e n   -   1 )  
                 e n d   i f  
                 D e v i c e G U I D   =   G e t V a r ( " d b _ D e v i c e G U I D " )  
                 R e g i o n G U I D   =   G e t V a r ( " d b _ R e g i o n G U I D " )  
                 c a l l   S e t V a r ( " R e g i o n G U I D " , R e g i o n G U I D )  
         E n d   I f  
          
         L o o k u p T a g   =   b F o u n d  
 e n d   f u n c t i o n  
  
 S u b   L o o k u p S i t e ( )  
         V a l i d   =   V a l i d a t e S i t e F i e l d s ( T r u e )  
          
         I f   V a l i d   T h e n  
                 ' L o o k u p   S i t e  
                 S i t e I D   =   G e t V a r ( " S i t e I D " )  
                 S i t e I D   =   T r i m ( S i t e I D )  
                 c a l l   S e t V a r ( " S i t e I D " ,   S i t e I D )  
                 '   D o   w e   n e e d   t o   c h e c k   r e g i o n   h e r e ?  
                  
                 ' S q l   =   " S E L E C T   R e l a y G u i d ,   O r g 1   a s   C o m p a n y   F R O M   R e l a y   W H E R E   b I s D e l   =   0   A N D   O r g 2   =   ' "   &   S i t e I D   &   " '   "  
                 ' c a l l   a L o g ( " S Q L   =   "   &   S q l )  
                 ' c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S q l ,   0 )  
                  
                 C o m p a n y G u i d   =   L o a d S i t e A d d r e s s ( S i t e I D )  
  
                 ' R e l a y G u i d   =   G e t V a r ( " d b _ R e l a y G U I D " )  
                 I f   N o t   I s B l a n k S t r i n g ( C o m p a n y G u i d )   T h e n  
                         ' C o m p a n y   =   G e t V a r ( " d b _ C o m p a n y " )  
                         ' c a l l   S e t V a r ( " C o m p a n y " ,   C o m p a n y )  
                         ' c a l l   S e t V a r ( " R e l a y G u i d " ,   R e l a y G u i d )  
                         ' C o m p a n y G u i d   =   L o a d S i t e A d d r e s s ( S i t e I D )  
                         c a l l   S e t V a r (   " C o m p a n y G u i d " ,   C o m p a n y G u i d   )  
                          
                          
                         c a l l   S e t S c r e e n ( " C o n f i r m J o b C r e a t e " )  
                 E l s e  
                         c a l l   S e t S c r e e n ( " C o n f i r m S i t e C r e a t e " )  
                 E n d   I f  
          
         E n d   I f  
          
 E n d   S u b  
  
 S u b   L o o k u p S i t e B y C u s t o m e r ( )  
         S i t e I D   =   G e t V a r ( " S i t e I D " )  
         c a l l   S e t V a r ( " P r e F i n d S i t e I D " ,   S i t e I D )  
         c a l l   S e t V a r ( " S i t e I D " ,   " " )  
         c a l l   C l e a r S e a r c h F i e l d s ( )  
         c a l l   S e t S c r e e n ( " L o o k u p S i t e B y C u s t o m e r " )  
 E n d   S u b  
  
 S u b   U s e S i t e ( )  
         V a l i d   =   V a l i d a t e S i t e F i e l d s ( T r u e )  
          
         I f   V a l i d   T h e n  
                 S i t e I D   =   G e t V a r ( " S i t e I D " )  
                 S i t e I D   =   T r i m ( S i t e I D )  
                 c a l l   S e t V a r ( " S i t e I D " ,   S i t e I D )  
                 C o m p a n y G u i d   =   L o a d S i t e A d d r e s s ( S i t e I D )  
                  
                 c a l l   S e t V a r (   " C o m p a n y G u i d " ,   C o m p a n y G u i d   )  
                 c a l l   S e t S c r e e n ( " C o n f i r m J o b C r e a t e " )  
         E n d   I f  
 E n d   S u b  
  
 F u n c t i o n   C r e a t e J o b ( )  
         J o b N u m b e r   =   G e t V a r ( " T i c k e t " )   &   " - "   &   G e t V a r ( " T a g I D " )  
         C o m p a n y G u i d   =   G e t V a r ( " C o m p a n y G u i d " )  
         c a l l   a l o g ( " C O M P A N Y G U I D   "   &   C o m p a n y G u i d   )  
         i f   I s B l a n k S t r i n g ( C o m p a n y G u i d )   t h e n  
                 J o b G u i d   =   A p p l i c a t i o n . R u n A p p C m d ( - 1 ,   " C r e a t e J o b " ,   J o b N u m b e r   ,   0 )  
         e l s e  
                 J o b G u i d   =   A p p l i c a t i o n . R u n A p p C m d ( - 1 ,   " C r e a t e J o b " ,   J o b N u m b e r   &   " , "   &   C o m p a n y G u i d ,   0 )  
         e n d   i f  
  
         ' S q l   =   " S E L E C T   T o p   1   R e g i o n G U I D   F r o m   P d b R e g i o n s "  
         ' c a l l   a L o g ( " S Q L   =   "   &   S q l )  
         ' c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S q l ,   0 )  
          
         R e g i o n G U I D   =   G e t V a r ( " R e g i o n G U I D " )  
          
         i f   ( I s E m p t y ( R e g i o n G U I D )   o r   I s N u l l ( R e g i o n G U I D ) )   t h e n  
                 R e g i o n G U I D   =   G e t V a r ( " R e g i o n G U I D L i s t " )  
         e n d   i f  
            
          
         c a l l   a l o g ( " R E G I O N G U I D   "   &   R e g i o n G U I D   )  
          
         A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   R e g i o n G U I D   =   ' "   &   R e g i o n G U I D   &   " '   w h e r e   J o b G U I D   =   ' "   &   J o b G U I D   &   " ' " )  
         R e p o r t T y p e   =   G e t V a r ( " R e p o r t T y p e " )  
         s e l e c t   c a s e   R e p o r t T y p e  
         c a s e   0  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' B a t t e r y   M a i n t e n a n c e   R e p o r t '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )  
         c a s e   1  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' S i t e   S c a n   R e p o r t '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )  
         c a s e   2  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' A P M   R e p o r t '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i   &   " ' " )  
         c a s e   3  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' A P S   R e p o r t '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )  
         c a s e   4  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' N X   2 2 5 - 6 0 0   R e p o r t '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )  
         c a s e   5  
          
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' 1 0 7 7 1   R e p o r t '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )  
         c a s e   6  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' T H E R M A L   P M V '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )          
         c a s e   7  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' 8 0 8 0 0   U P S '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )    
         c a s e   8  
                 A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   P d b J o b   S e t   D e s c r i p t i o n   =   ' E H S   S i t e   S u r v e y '   w h e r e   J o b G U I D   =   ' "   &   J o b G u i d   &   " ' " )    
         e n d   s e l e c t  
          
         C r e a t e J o b   =   J o b G u i d  
 E n d   F u n c t i o n  
  
 S u b   A d d A s s e t T o J o b ( J o b G u i d ,   R e l a y G u i d )  
  
         c a l l   a L o g ( " A d d A s s e t T o J o b :   J o b G u i d = "   &   J o b G u i d   &   " , R e l a y G u i d = "   &   R e l a y G u i d )  
  
         I f   N o t   I s B l a n k S t r i n g ( J o b G u i d )   A n d   N o t   I s B l a n k S t r i n g ( R e l a y G u i d )   T h e n          
                 c a l l   A p p l i c a t i o n . O p e n J o b ( J o b G u i d )      
                 R e t V a l   =   A p p l i c a t i o n . R u n A p p C m d ( - 1 ,   " A d d A s s e t T o J o b " ,   R e l a y G u i d   &   " , "   &   J o b G u i d   &   " , 1 " ,   0 )                  
         E n d   I f  
  
 E n d   S u b  
  
 S u b   C r e a t e J o b A d d A s s e t ( )  
  
         I f   V a l i d a t e C u s t o m e r F i e l d   T h e n  
                 J o b G u i d   =   C r e a t e J o b ( )  
                 R e l a y G u i d   =   G e t V a r ( " R e l a y G u i d " )  
                  
                 I f   I s B l a n k S t r i n g ( R e l a y G u i d )   T h e n  
                         c a l l   A p p l i c a t i o n . O p e n J o b ( J o b G u i d )      
                         c a l l   A d d N e w A s s e t T o J o b ( )  
                 E l s e  
                         c a l l   A d d A s s e t T o J o b ( J o b G u i d ,   R e l a y G u i d )  
                 E n d   I f          
                  
                 c a l l   E x i t P o p u p ( )  
                                  
                 F o r m . R e f r e s h   =   1  
         E n d   I f  
          
 E n d   S u b  
  
 S u b   C r e a t e J o b C r e a t e A s s e t ( )  
          
         I f   V a l i d a t e C u s t o m e r F i e l d   T h e n  
                 '   A d d   s i t e   a d d r e s s .    
                 c a l l   A p p l i c a t i o n . R u n A p p C m d ( F o r m . F o r m I d ,   " A d d P l a n t A d d r e s s " ,   " C u s t o m e r , A d d r e s s , A d d r e s s 2 , C i t y , S t a t e , C o u n t r y , Z i p , S i t e I D , F i r s t N a m e , L a s t N a m e " ,   1 )  
                  
                 C o m p a n y G u i d   =   L o a d S i t e A d d r e s s ( G e t V a r ( " S i t e I D " ) )  
                  
                 i f   N O T   I s B l a n k S t r i n g ( C o m p a n y G u i d )   t h e n  
                         c a l l   S e t V a r ( " C o m p a n y G u i d " ,   C o m p a n y G u i d )  
                 e n d   i f  
                          
                 J o b G u i d   =   C r e a t e J o b ( )  
                 c a l l   A p p l i c a t i o n . O p e n J o b ( J o b G u i d )              
                  
                 c a l l   A d d N e w A s s e t T o J o b ( )  
                  
                 c a l l   E x i t P o p u p ( )  
                  
                 F o r m . R e f r e s h   =   1  
         E n d   I f  
          
 E n d   S u b  
  
 S u b   A d d N e w A s s e t T o J o b ( )  
         O r g 1   =   G e t V a r ( " C u s t o m e r " )  
         O r g 2   =   G e t V a r ( " S i t e I D " )   &   " - "   &   G e t V a r ( " A d d r e s s " )  
         O r g 3   =   " < N o n e > "  
         O r g 4   =   G e t V a r ( " T a g I D " )  
          
         I f   O r g 1   =   " "   T h e n   O r g 1   =   " < N o n e > "  
          
         c a l l   a L o g ( " O r g 1   =   "   &   O r g 1 )  
         c a l l   a L o g ( " O r g 2   =   "   &   O r g 2 )  
         c a l l   a L o g ( " O r g 3   =   "   &   O r g 3 )  
         c a l l   a L o g ( " O r g 4   =   "   &   O r g 4 )  
          
         R e p o r t T y p e   =   G e t V a r ( " R e p o r t T y p e " )  
          
         s e l e c t   c a s e   R e p o r t T y p e  
                 c a s e   0  
                         F o r m G U I D   =   " B X I R A A A I 3 N H J 4 U S Q Y I S N "  
                 c a s e   1  
                         F o r m G U I D   =   " A U Q 3 A A A I R S U 2 J A S T 4 X S P "  
                 c a s e   2  
                         F o r m G U I D   =   " A V Y 3 A A A C Z B F P 2 7 P U 6 F 2 S "  
                 c a s e   3  
                         F o r m G U I D   =   " A V Y 3 A A A N D G K P J Q Y F C X S T "  
                 c a s e   4  
                         F o r m G U I D   =   " A V Y 3 A A A A D G 4 L K R G I E F C T "  
                 c a s e   5  
                         F o r m G U I D   =   " A V Y 3 A A A D S V 7 S H 2 M B M O C S "  
                 c a s e   6  
                         F o r m G U I D   =   " A 2 Y W A A A G N Z 2 B I S 6 P 7 U C U "  
                 c a s e   7  
                         F o r m G U I D   =   " A 2 Y W A A A O M T P Y L 2 I H Q J S W "  
                 c a s e   8  
                         F o r m G U I D   =   " A V Y 3 A A A O J T 2 J Q 6 B C E B S X "                  
                          
         e n d   s e l e c t  
                          
                          
         c a l l   A p p l i c a t i o n . A d d N e w F o r m ( O r g 1 ,   O r g 2 ,   O r g 3 ,   O r g 4 ,   F o r m G u i d )  
          
 E n d   S u b  
  
  
 S u b   C l e a r A d d r e s s F i e l d s ( )  
         C a l l   S e t V a r ( " C u s t o m e r " ,   " " )  
         C a l l   S e t V a r ( " A d d r e s s " ,   " " )  
         C a l l   S e t V a r ( " A d d r e s s 2 " ,   " " )  
         C a l l   S e t V a r ( " C i t y " ,   " " )  
         C a l l   S e t V a r ( " Z i p " ,   " " )  
         C a l l   S e t V a r ( " S t a t e " ,   " " )  
         C a l l   S e t V a r ( " C o u n t r y " ,   " " )          
 E n d   S u b  
  
 S u b   E x i t P o p u p ( )  
         F o r m . F o r m D l g R e t u r n V a l u e   =   " O K "  
         F o r m . F o r m D l g R e t u r n   =   " O K "  
 E n d   S u b  
  
 S u b   L o a d T a g F r o m F i l e ( )  
          
          
          
         S e t   o b j F S O   =   C r e a t e O b j e c t ( " S c r i p t i n g . F i l e S y s t e m O b j e c t " )  
         S e t   o b j F i l e   =   o b j F S O . O p e n T e x t F i l e ( " C : \ m o b i l e \ P D B _ I n s p e c t i o n . t x t " ,   1 )  
         s t r C o n t e n t s   =   o b j F i l e . R e a d A l l  
        
         o b j F i l e . C l o s e  
         S e t   o b j F i l e   =   N o t h i n g  
          
          
          
 E n d   S u b  
  
 F U N C T I O N   L o a d S i t e A d d r e s s ( S i t e I D )  
         S Q L   =   " S E L E C T   T O P   1   C . [ N a m e ] ,   C . C o m p a n y G U I D ,   A . A d d r G u i d ,   A . A d d r L n 1 ,   A . A d d r L n 2 ,   A . C i t y ,   A . S t a t e ,   A . C o u n t r y ,   A . Z i p ,   A . F i r s t N a m e ,   A . L a s t N a m e   "   & _  
                     "   F R O M   P d b A d d r I n f o   A   L E F T   J O I N   P d b A d d r H e a d e r   C   o n   A . C o m p a n y G u i d   =   C . C o m p a n y G U I D   "   & _  
                     "   W H E R E   A . [ T y p e ]   =   2   A n d   A . [ b I s D e l ]   =   0   A n d   ( A . A d d r N e u t r a l I d   =   ' "   &   S i t e I D   &   " '   O R   A . A d d r N e u t r a l I d   =   ' "   &   S i t e I D   &   " - 2 ' ) "  
          
         C a l l   S e t V a r ( " d b _ A d d r G u i d " ,   E m p t y )              
         c a l l   a L o g ( " S Q L = "   &   S Q L )  
          
         C a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . f o r m I d ,   S Q L ,   0 )  
          
         A d d r G u i d   =   G e t V a r ( " d b _ A d d r G u i d " )  
         I f   N o t   I s B l a n k S t r i n g ( A d d r G u i d )   T h e n  
                 C o m p a n y   =   G e t V a r ( " d b _ N a m e " )  
                 C o m p a n y G u i d   =   G e t V a r ( " d b _ C o m p a n y G u i d " )  
                 A d d r L n 1   =   G e t V a r ( " d b _ A d d r L n 1 " )  
                 A d d r L n 2   =   G e t V a r ( " d b _ A d d r L n 2 " )  
                 C i t y   =   G e t V a r ( " d b _ C i t y " )  
                 S t a t e   =   G e t V a r ( " d b _ S t a t e " )  
                 C o u n t r y   =   G e t V a r ( " d b _ C o u n t r y " )  
                 Z i p   =   G e t V a r ( " d b _ Z i p " )  
                 F i r s t N a m e   =   G e t V a r ( " d b _ F i r s t N a m e " )  
                 L a s t N a m e   =   G e t V a r ( " d b _ L a s t N a m e " )  
                  
                 C a l l   S e t V a r ( " C u s t o m e r " ,   C o m p a n y )  
                 C a l l   S e t V a r ( " C o m p a n y " ,   C o m p a n y )  
                 C a l l   S e t V a r ( " A d d r e s s " ,   A d d r L n 1 )  
                 C a l l   S e t V a r ( " A d d r e s s 2 " ,   A d d r L n 2 )  
                 C a l l   S e t V a r ( " C i t y " ,   C i t y )  
                 C a l l   S e t V a r ( " S t a t e " ,   S t a t e )  
                 C a l l   S e t V a r ( " t x t _ G e t S t a t e " ,   S t a t e )           '  
                 C a l l   S e t V a r ( " Z i p " ,   Z i p )  
                 C a l l   S e t V a r ( " C o u n t r y " ,   C o u n t r y )  
                 c a l l   S e t V a r ( " F i r s t N a m e A t t n " ,   F i r s t N a m e )  
                 c a l l   S e t V a r ( " L a s t N a m e A t t n " ,   L a s t N a m e )  
                 c a l l   S e t V a r ( " C u s t o m e r A t t n " ,   F i r s t N a m e   &   "   "   &   L a s t N a m e )  
                  
                 L o a d S i t e A d d r e s s   =   C o m p a n y G u i d  
                 c a l l   a L o g ( " F o u n d   S i t e   A d d r e s s :   "   &   A d d r G u i d   &   " ,   "   &   C o m p a n y G u i d   &   " ,   "   &   C o m p a n y   &   " ,   "   &   A d d r L n 1   &   " ,   "   &   A d d r L n 2   &   " ,   "   &   C i t y   &   " ,   "   &   S t a t e   &   " ,   "   &   Z i p   &   " ,   "   &   F i r s t N a m e   &   " ,   "   &   L a s t N a m e )  
         E l s e  
                 c a l l   a L o g ( " D i d   n o t   f i n d   s i t e   a d d r e s s " )  
         E n d   I f          
          
          
 E n d   F U N C T I O N  
  
 S u b   C h a n g e F o r m i n T a g ( T a g )  
          
         r e s u l t   =   m s g b o x ( " Y o u   a r e   a t t e m p t i n g   t o   m o d i f y   t h e   e q u i p m e n t   t y p e   o f   t h i s   t a g . "   +   v b C r L f   +   v b C r L f   + " D o   y o u   w i s h   t o   c o n t i n u e ? " , v b Y e s N o + v b E x c l a m a t i o n + v b D e f a u l t B u t t o n 2 , " W a r n i n g " )  
          
         i f   r e s u l t   =   6   t h e n  
                  
                 R e p o r t T y p e   =   G e t V a r ( " R e p o r t T y p e " )  
          
                 s e l e c t   c a s e   R e p o r t T y p e  
                         c a s e   0  
                                 F o r m G U I D   =   " B X I R A A A I 3 N H J 4 U S Q Y I S N "  
                         c a s e   1  
                                 F o r m G U I D   =   " A U Q 3 A A A I R S U 2 J A S T 4 X S P "  
                         c a s e   2  
                                 F o r m G U I D   =   " A V Y 3 A A A C Z B F P 2 7 P U 6 F 2 S "  
                         c a s e   3  
                                 F o r m G U I D   =   " A V Y 3 A A A N D G K P J Q Y F C X S T "  
                         c a s e   4  
                                 F o r m G U I D   =   " A V Y 3 A A A A D G 4 L K R G I E F C T "  
                         c a s e   5  
                                 F o r m G U I D   =   " A V Y 3 A A A D S V 7 S H 2 M B M O C S "  
                          
                 e n d   s e l e c t  
                  
                 S q l   =   " S E L E C T   D e v i c e N a m e   F R O M   D e v i c e _ T y p e   W H E R E   b I s D e l   =   0   A N D   D e v i c e G U I D   =   ' "   &   F o r m G U I D   &   " '   "  
                 c a l l   a L o g ( " S Q L   =   "   &   S q l )  
                 c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S q l ,   0 )  
                 R e l a y N a m e   =   G e t V a r ( " d b _ D e v i c e N a m e " )  
  
                 c a l l   L o o k u p T a g ( T a g ,   S i t e I D ,   C o m p a n y ,   R e l a y G U I D ,   D e v i c e G U I D ,   R e g i o n G U I D )      
                 c a l l   S e t V a r ( " S i t e I D " ,   S i t e I D )  
                 c a l l   S e t V a r ( " T a g I D " ,   T a g )  
                 c a l l   S e t V a r ( " C u s t o m e r " ,   C o m p a n y )              
                 c a l l   A d d N e w A s s e t T o J o b ( )  
                 ' A p p l i c a t i o n . E x e c u t e S q l ( " U p d a t e   R e l a y   S e t   D e v i c e G U I D   =   ' "   &   F o r m G U I D   &   " ' ,   R e l a y N a m e   =   ' "   &   R e l a y N a m e   &   " '   W H E R E   b I s D e l   =   0   A N D   O r g 3   =   ' "   &   T a g   &   " '   " )  
  
                 E r r M s g   =   " "  
                 c a l l   S e t V a r ( " R e p o r t T y p e E r r " ,   E r r M s g )  
                 c a l l   S e t V a r ( " U p d a t e T a g " ,   0 )  
  
                  
         e n d   i f  
          
          
 E n d   S u b  
  
     mE#џўџD a t a џўџ     џўџ  џџ  OPdbFormsEmbeddedWsts5;iE#      ?  ?                           џўџN e w J o b S u b f o r m C r e a t e N e w џўџ tE#џџџ     p  0                      iF#	             џџ   OOptDrawBorderData                                џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџџџ   OFormsToolEmbeddedWs76       5>        p  0  lE#  џўџ_ _ N e w J o b S u b f o r m C r e a t e N e w      ЭЭЭЭ   ts5;iE#      ?  ?                           џўџN e w J o b S u b f o r m C o n f i r m T a g C r e a t e џўџ tE#џџџ     p  0                      iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџ76       5>        p  0  lE#  џўџ_ _ N e w J o b S u b f o r m C o n f i r m T a g C r e a t e      ЭЭЭЭ   ts5;iE#      ?  ?                           џўџN e w J o b S u b f o r m C o n f i r m S i t e C r e a t e џўџ tE#џџџ     p  0                      iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџ76       5>        p  0  lE#  џўџ _ _ N e w J o b S u b f o r m C o n f i r m S i t e C r e a t e      ЭЭЭЭ   ts5;iE#      ?  ?                           џўџN e w J o b S u b f o r m C o n f i r m J o b C r e a t e џўџ tE#џџџ     p  0                      iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџ76       5>        p  0  lE#  џўџ_ _ N e w J o b S u b f o r m C o n f i r m J o b C r e a t e      ЭЭЭЭ   ts5;iE#      ?  ?                           џўџN e w J o b S u b f o r m L o o k u p џўџ tE#џџџ     p  0                      iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџ76       5>        p  0  lE#  џўџ_ _ N e w J o b S u b f o r m L o o k u p      ЭЭЭЭ   ts5;iE#      ?  ?                           џўџN e w J o b S u b f o r m L o o k u p S i t e џўџ tE#џџџ     p  0                      iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџ76       5>        p  0  lE#  џўџ_ _ N e w J o b S u b f o r m L o o k u p S i t e      ЭЭЭЭ   ts5;iE#      ?  ?                           џўџ!N e w J o b S u b f o r m L o o k u p S i t e B y C u s t o m e r џўџ tE#џџџ     p  0                      iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџ76       5>        p  0  lE#  џўџ#_ _ N e w J o b S u b f o r m L o o k u p S i t e B y C u s t o m e r      ЭЭЭЭ   џџ  OPdbDrawText   tE#џџџ     з        5            iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџџџ   OPdbFormsToolTextC76        77    џџ   OPdbFormsTagInfo        џўџ       5  f  I  lE# 0   E M E A   N e w   J o b   v M a r 0 7 - 2 0 1 6 џўџ      D a    qE  џўџE M E A   N e w   J o b   v M a r 0 7 - 2 0 1 6       Р=                                       V#             џўџA r i a l џџџ             џўџD a t e M o d i f i e d        U`їМф@џўџD a t e C r e a t e d        сznѓйу@џўџ	D a t e S y n c h        сznѓйу@џўџb I s D e l         џўџ
D e l e t e I n f o    џўџ џўџ
b I s R e t i r e d        џўџb O p t i m a F o r m        џўџb S u g g e s t U p d a t e        џўџ	L a s t M o d B y    џўџ џўџC r e a t o r G u i d    џўџ џўџT e m p l a t e s U s e d    џўџџKU s e r   D a t a | O p t i m a   D a t a   -   D o   N o t   C h a n g e | C u s t o m   F i e l d   E n t r y | N e w J o b S u b f o r m C o n f i r m J o b C r e a t e | N e w J o b S u b f o r m C r e a t e N e w | N e w J o b S u b f o r m C o n f i r m T a g C r e a t e | N e w J o b S u b f o r m C o n f i r m S i t e C r e a t e | N e w J o b S u b f o r m C o n f i r m J o b C r e a t e | N e w J o b S u b f o r m C o n f i r m T a g C r e a t e | N e w J o b S u b f o r m C r e a t e N e w | N e w J o b S u b f o r m L o o k u p | N e w J o b S u b f o r m L o o k u p S i t e | N e w J o b S u b f o r m L o o k u p S i t e B y C u s t o m e r џўџE x t e r n a l T e m p l a t e s    џўџ џўџT e m p l a t e F a m i l y    џўџ џўџT e m p l a t e M a n u    џўџ џўџb I s L o c a l F o r m         џўџS y n c h W r i t e D a t e        №Э+Кф@џўџS t a t F l g         џўџb L o c k e d        џўџL o c k I n f o    џўџ 
   џўџA M I W A A A J 2 N N G T Q Q U W H K R 
 T e m p l a t e :   N e w J o b S u b f o r m L o o k u p S i t e B y C u s t o m e r 
 D a t e   C r e a t e d :   ! 2 0 1 1 , 0 3 , 1 5 , 1 5 , 0 8 , 1 8 
 D a t e   M o d i f i e d :   ! 2 0 1 5 , 0 7 , 2 0 , 0 9 , 1 8 , 1 8    џўџT e m p l a t e G U I D    џўџA M I W A A A J 2 N N G T Q Q U W H K R џўџN a m e    џўџ!N e w J o b S u b f o r m L o o k u p S i t e B y C u s t o m e r џўџ
D e f i n i t i o n    }H  optform          SAVDOUT}s65>?џўџ   А@  `@                =                         џџ  OOptDrawCtrlScriptmE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџџs u b   C h a n g e A d d r e s s ( N d x )  
         S i t e I D   =   G e t N d x V a r ( " d b _ A d d r N e u t r a l I d " , N d x )  
         N a m e   =   G e t N d x V a r ( " d b _ N a m e " , N d x )  
         A d d r L n 1   =   G e t N d x V a r ( " d b _ A d d r L n 1 " , N d x )  
         A d d r L n 2   =   G e t N d x V a r ( " d b _ A d d r L n 2 " , N d x )  
         C i t y   =   G e t N d x V a r ( " d b _ C i t y " , N d x )  
         S t a t e   =   G e t N d x V a r ( " d b _ S t a t e " , N d x )  
         Z i p   =   G e t N d x V a r ( " d b _ Z i p " , N d x )  
         i f   N O T   I s B l a n k S t r i n g ( A d d r L n 2 )   t h e n  
         A d d r 2   =   A d d r L n 2   &   c h r ( 1 0 )  
         e n d   i f  
         A d d r e s s   =   N a m e   &   c h r ( 1 0 )   &   A d d r L n 1   &   c h r ( 1 0 )   &   A d d r 2   &   C i t y   &   " ,   "   &   S t a t e   & "   "   &   Z i p  
         c a l l   S e t V a r ( " S i t e I d " ,   R e p l a c e ( S i t e I D , " - 2 " , " " ) )  
         c a l l   S e t V a r ( " C u s t o m e r " ,   N a m e )  
         c a l l   S e t V a r ( " a d d r d i s p " ,   A d d r e s s )  
 e n d   s u b  
  
 s u b   C l e a r S e a r c h F i e l d s ( )  
         c a l l   S e t V a r ( " C u s t o m e r " ,   " " )  
         c a l l   S e t V a r ( " C u r r e n t N d x " ,   0 )  
         c a l l   S e t V a r ( " R e t u r n e d " ,   0 )  
         c a l l   S e t V a r ( " a d d r d i s p " ,   " " )  
 e n d   s u b  
     mE#џўџD a t a џўџ            РРР џўџ0                џўџ             mE#         $2         џўџ       РРР p  0  0  p                mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџџs u b   C h a n g e A d d r e s s ( N d x )  
         S i t e I D   =   G e t N d x V a r ( " d b _ A d d r N e u t r a l I d " , N d x )  
         N a m e   =   G e t N d x V a r ( " d b _ N a m e " , N d x )  
         A d d r L n 1   =   G e t N d x V a r ( " d b _ A d d r L n 1 " , N d x )  
         A d d r L n 2   =   G e t N d x V a r ( " d b _ A d d r L n 2 " , N d x )  
         C i t y   =   G e t N d x V a r ( " d b _ C i t y " , N d x )  
         S t a t e   =   G e t N d x V a r ( " d b _ S t a t e " , N d x )  
         Z i p   =   G e t N d x V a r ( " d b _ Z i p " , N d x )  
         i f   N O T   I s B l a n k S t r i n g ( A d d r L n 2 )   t h e n  
         A d d r 2   =   A d d r L n 2   &   c h r ( 1 0 )  
         e n d   i f  
         A d d r e s s   =   N a m e   &   c h r ( 1 0 )   &   A d d r L n 1   &   c h r ( 1 0 )   &   A d d r 2   &   C i t y   &   " ,   "   &   S t a t e   & "   "   &   Z i p  
         c a l l   S e t V a r ( " S i t e I d " ,   R e p l a c e ( S i t e I D , " - 2 " , " " ) )  
         c a l l   S e t V a r ( " C u s t o m e r " ,   N a m e )  
         c a l l   S e t V a r ( " a d d r d i s p " ,   A d d r e s s )  
 e n d   s u b  
  
 s u b   C l e a r S e a r c h F i e l d s ( )  
         c a l l   S e t V a r ( " C u s t o m e r " ,   " " )  
         c a l l   S e t V a r ( " C u r r e n t N d x " ,   0 )  
         c a l l   S e t V a r ( " R e t u r n e d " ,   0 )  
         c a l l   S e t V a r ( " a d d r d i s p " ,   " " )  
 e n d   s u b  
     mE#џўџD a t a џўџ     џўџ  џџ  OPdbDrawText   tE#РРР       #   d      d           iF#	             џџ   OOptDrawBorderData                                џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџџџ   OPdbFormsToolText76        77    џџ   OPdbFormsTagInfo        џўџ     d      u  7   lE#     џўџM a i n E r r     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ 
      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ       Р=                                       ЭV#             џўџA r i a l џџџ                tE#РРР     з      ;  №   ;          iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     ;  №       lE#     џўџT a g I D      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     }      Д   №   Д           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Д   №   1    lE#    I n s t a n c e : џўџ      ЭЭЭЭ   qE  џўџ	I n s t a n c e :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     }      Д     Д           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Д     1  6  lE#    S i t e   I D : џўџ      ЭЭЭЭ   qE  џўџS i t e   I D :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     }      Д   Ш   Д           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Д   Ш   1  ц   lE# 
   T a s k : џўџ      ЭЭЭЭ   qE  џўџT a s k :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#џџџ     n   2     ж        iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ       ж      lE#    B a c k џўџL o o k u p S i t e B y C u s t o m e r B a c k     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџzP r e F i n d S i t e I D   =   G e t V a r ( " P r e F i n d S i t e I D " )  
 c a l l   S e t V a r ( " S i t e I D " ,   P r e F i n d S i t e I D )  
 c a l l   S e t V a r ( " S i t e I D E r r " ,   " " )  
 c a l l   G o B a c k ( )     mE#џўџO n   E n t e r џўџ      mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџB a c k     9у=                                     ЭV#              џўџA r i a l џџџ                tE#РРР     з      ;    ;          iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     ;      6  lE#     џўџS i t e I D      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     з      ;  Ш   ;          iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     ;  Ш     ц   lE#     џўџT i c k e t      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     щ  #   K   2   K           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     K   2   4  U   lE# .   L O O K U P   S I T E   B Y   C U S T O M E R џўџL o o k u p S i t e B y C u s t o m e r H e a d i n g      ЭЭЭЭ   qE  џўџL O O K U P   S I T E   B Y   C U S T O M E R     ЋЊ*>                                      ЭV#             џўџA r i a l џџџ                tE#џџџ     з      ;  @  ;          iF#	                                       џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     ;  @    ^  lE#     џўџC u s t o m e r       ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#џџџ     }   (     ;        iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ       ;    c  lE#    S e a r c h џўџ	S e a r c h B u t     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџџC u s t o m e r   =   G e t V a r ( " C u s t o m e r " )  
 S Q L   =   " S E L E C T   T O P   3 0   C . [ N a m e ] ,   A . A d d r N e u t r a l I d ,   A . A d d r L n 1 ,   A . A d d r L n 2 ,   A . C i t y ,   A . S t a t e ,   A . Z i p ,   A . F i r s t N a m e ,   A . L a s t N a m e   "   & _  
                     "   F R O M   P d b A d d r I n f o   A   I N N E R   J O I N   P d b A d d r H e a d e r   C   o n   A . C o m p a n y G u i d   =   C . C o m p a n y G U I D   W H E R E   C . [ N a m e ]   L I K E   ' % % "   &   C u s t o m e r   &   " % % ' "  
                      
 c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d , S Q L , 1 )  
  
 c a l l   S e t V a r ( " S i t e I d " , " " )  
                      
 N u m R e c o r d s   =   G e t V a r ( " d b _ N u m R e c o r d s " )  
 i f   N u m R e c o r d s   >   0   t h e n  
         C h a n g e A d d r e s s ( 1 )  
         . C u r r e n t N d x   =   1  
         c a l l   S e t V a r ( " R e t u r n e d " ,   N u m R e c o r d s )  
 e l s e  
         c a l l   S e t V a r ( " R e t u r n e d " ,   0 )  
         c a l l   S e t V a r ( " a d d r d i s p " ,   " " )  
         . C u r r e n t N d x   =   0  
 e n d   i f  
  
                      
  
                         mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџS e a r c h     9у=                                     ЭV#              џўџA r i a l џџџ                tE#џџџ        2   Є  ж  Є      iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     Є  ж  :    lE#    U s e   S i t e џўџU s e S i t e B u t t o n     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџC a l l   U s e S i t e ( )     mE#џўџO n   E n t e r џўџ      mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџU s e   S i t e     9у=                                     ЭV#              џўџA r i a l џџџ                tE#џџџ     -      E  h  E      iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     E  h  r  |  lE#    < < џўџP r e v i o u s     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџei f   . C u r r e n t N d x   -   1   >   0   t h e n  
 . C u r r e n t N d x   =   . C u r r e n t N d x   -   1  
 c a l l   C h a n g e A d d r e s s ( . C u r r e n t N d x )  
 e n d   i f  
     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ< <       Р=                                     ЭV#             џўџA r i a l џџџ                tE#РРР     K      %  #   %          iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџ76        77            џўџ     %  #   p  7   lE#  џўџ      ЭЭЭЭ   qE  џўџ       Р=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     K                       iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџ76        77            џўџ           _   (   lE#  џўџ      ЭЭЭЭ   qE  џўџ       Р=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     '  P                 iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ         :  б  lE#     џўџa d d r d i s p      ЭЭЭЭ   qE  џўџ     ЋЊЊ=                                        ЭV#              џўџA r i a l џџџ                tE#џџџ     -        h        iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ       h  :  |  lE#    > I џўџL a s t     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџH. C u r r e n t N d x   =   G e t V a r ( " R e t u r n e d " )  
 c a l l   C h a n g e A d d r e s s ( G e t V a r ( " R e t u r n e d " ) )     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ> I       Р=                                     ЭV#             џўџA r i a l џџџ                tE#џџџ     -      л  h  л      iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     л  h    |  lE#    > > џўџN e x t     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџwi f   . C u r r e n t N d x   +   1   < =   G e t V a r ( " R e t u r n e d " )   t h e n  
 . C u r r e n t N d x   =   . C u r r e n t N d x   +   1  
 c a l l   C h a n g e A d d r e s s ( . C u r r e n t N d x )  
 e n d   i f  
     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ> >       Р=                                     ЭV#             џўџA r i a l џџџ             џџjE OOptDrawEdittE#РРР           И  h  И          iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџџџ   OPdbFormsToolNumericЉ76       78            џўџ     И  h  Ь  |  lE#         џўџR e t u r n e d     mE#џўџO n   I n i t i a l   U p d a t e џўџ' . t h i s ! v i s i b l e   =   f a l s e     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ0      Р=                                       ЭV#             џўџA r i a l џџџ             jE                    џўџ% . 6 g               tE#џџџ     -        h        iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ       h  @  |  lE#    I < џўџF i r s t     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџXi f   G e t V a r ( " R e t u r n e d " )   >   0   t h e n  
         c a l l   C h a n g e A d d r e s s ( 1 )  
         . C u r r e n t N d x   =   1  
 e n d   i f  
     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџI <       Р=                                     ЭV#             џўџA r i a l џџџ                tE#РРР             h            iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ       h  И  |  lE#    o f џўџ      ЭЭЭЭ   qE  џўџo f       Р=                                      ЭV#             џўџA r i a l џџџ             ЃtE#РРР             h            iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџЉЉ76       78            џўџ       h    |  lE#     џўџ
C u r r e n t N d x     mE#џўџO n   I n i t i a l   U p d a t e џўџ' . t h i s ! v i s i b l e   =   f a l s e  
     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ0      Р=                                      ЭV#             џўџA r i a l џџџ             jE            џўџ% d               tE#РРР     }      Д   @  Д           iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Д   @  1  ^  lE#    S i t e   O w n e r : џўџ      ЭЭЭЭ   qE  џўџS i t e   O w n e r :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     O                   iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ         k  6  lE#     џўџ	S i t e I D E r r      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#џ            џўџA r i a l џџџ             џўџD a t e M o d i f i e d        NhЌф@џўџD a t e C r e a t e d        IР7/4еу@џўџ	D a t e S y n c h        IР7/4еу@џўџb I s D e l         џўџ
D e l e t e I n f o    џўџ џўџ
b I s R e t i r e d        џўџb O p t i m a F o r m        џўџb S u g g e s t U p d a t e        џўџ	L a s t M o d B y    џўџ џўџC r e a t o r G u i d    џўџ џўџT e m p l a t e s U s e d    џўџ8U s e r   D a t a | O p t i m a   D a t a   -   D o   N o t   C h a n g e | C u s t o m   F i e l d   E n t r y џўџE x t e r n a l T e m p l a t e s    џўџ џўџT e m p l a t e F a m i l y    џўџ џўџT e m p l a t e M a n u    џўџ џўџb I s L o c a l F o r m         џўџS y n c h W r i t e D a t e        Gь;ЖТ+п@џўџS t a t F l g         џўџb L o c k e d        џўџL o c k I n f o    џўџ џўџA Q Q B A A A O L O Y F D 4 7 B W X C B 
 T e m p l a t e :   O p t i m a   D a t a   -   D o   N o t   C h a n g e 
 D a t e   C r e a t e d :   ! 2 0 0 4 , 0 9 , 3 0 , 0 9 , 4 2 , 0 6 
 D a t e   M o d i f i e d :   ! 2 0 1 5 , 1 0 , 0 7 , 2 1 , 1 8 , 3 4    џўџT e m p l a t e G U I D    џўџA Q Q B A A A O L O Y F D 4 7 B W X C B џўџN a m e    џўџO p t i m a   D a t a   -   D o   N o t   C h a n g e џўџ
D e f i n i t i o n    Ѕг optform          SAVDOPT|s65>?џўџ ЭЬЬ=ЭЬЬ=                =                         џџ  OOptDrawCtrlScriptmE#џўџO n   I n i t i a l   U p d a t e џўџџv e r s i o n   =   g e t V e r s i o n  
 c a l l   S e t V a r ( " P d b V e r s i o n " ,   v e r s i o n )  
 . D e c l a r e V a r ( " P d b D e f a u l t M e t r i c " )  
 . D e c l a r e V a r ( " U s e M e t r i c " )  
 . D e c l a r e V a r ( " L a s t U s e M e t r i c " )  
 . D e c l a r e V a r ( " f o o t _ m e t e r _ s t r " )  
 . D e c l a r e V a r ( " i n c h _ c m _ s t r " )  
 . D e c l a r e V a r ( " l c _ f o o t _ m e t e r _ s t r " )  
 . D e c l a r e V a r ( " l c _ i n c h _ c m _ s t r " )  
 . D e c l a r e V a r ( " b N a r r o w F o r m a t " )  
 c a l l   S e t T a g I f E m p t y ( " D e f a u l t F r e q " , 6 0 )  
 o n   e r r o r   r e s u m e   n e x t    
 c a l l   A p p l i c a t i o n . A d d S a v e d V a r ( F o r m . F o r m I d , " U s e M e t r i c " )  
 c a l l   A p p l i c a t i o n . A d d S a v e d V a r ( F o r m . F o r m I d , " A s s e t I D _ S N " )  
 o n   e r r o r   g o t o   0  
 . D e c l a r e V a r ( " T e m p e r a t u r e C " )  
       mE#џўџ	O n   U p d a t e џўџџ'  
 '   S e t u p   m e t r i c   /   i m p e r i a l   v a r i a b l e s  
 '  
 i f   I s E m p t y (   . U s e M e t r i c )     t h e n   . U s e M e t r i c   =   . P d b D e f a u l t M e t r i c  
 i f   . U s e M e t r i c   < >   . L a s t U s e M e t r i c     o r   I s E m p t y (   . L a s t U s e M e t r i c   )   t h e n    
         . L a s t U s e M e t r i c   =   . U s e M e t r i c  
         i f   . U s e M e t r i c   t h e n    
                 t m p _ i n c h _ c m _ s t r   =   " c m "  
                 t m p _ f o o t _ m e t e r _ s t r   =   " M E T E R S "    
         e l s e    
                 t m p _ i n c h _ c m _ s t r   =   " I N C H E S "  
                 t m p _ f o o t _ m e t e r _ s t r   =   " F E E T "  
         e n d   i f    
         . f o o t _ m e t e r _ s t r   =     T r a n s l a t e (   t m p _ f o o t _ m e t e r _ s t r   )    
         . i n c h _ c m _ s t r   =     T r a n s l a t e (   t m p _ i n c h _ c m _ s t r )  
         . l c _ f o o t _ m e t e r _ s t r   =     T r a n s l a t e (   L C a s e (   t m p _ f o o t _ m e t e r _ s t r   )   )    
         . l c _ i n c h _ c m _ s t r   =     T r a n s l a t e ( L C a s e (   t m p _ i n c h _ c m _ s t r ) )  
          
 e n d   i f    
  
  
  
      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџџџџЁ$ '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e G r a p h O p t i o n s ( )  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 ' B a c k w a r d s   c o m p a t i b i l i t y ,   P r e f i x   p a r a m   w a s   n e v e r   u s e d .  
 s u b   U p d a t e G r a p h O p t i o n s ( P r e f i x ,   C h a r t ,   N u m T r a c e s )  
         c a l l   U p d a t e G r a p h O p t i o n s E x ( C h a r t ,   N u m T r a c e s )  
 e n d   s u b  
 s u b   U p d a t e G r a p h O p t i o n s E x ( C h a r t ,   N u m T r a c e s )  
         i f   G e t V a r ( C h a r t & " _ I n i t " )   =   1   t h e n    
                 i f   R u n T a g C m d ( C h a r t , " I s C h a r t " )   =   " V a l i d C h a r t "   t h e n  
                         c a l l   S e t V a r ( C h a r t & " _ I n i t " ,   0 )                    
                         i f (   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e X " )   =   0   o r   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e Y " )   =   0   )   t h e n  
                                 f o r   N d x   =   1   t o   N u m T r a c e s  
                                         G r a p h M i n X V a l   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M i n X V a l " )  
                                         G r a p h M a x X V a l   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M a x X V a l " )  
                                         G r a p h M i n Y V a l   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M i n Y V a l " )  
                                         G r a p h M a x Y V a l   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M a x Y V a l " )  
                                         i f   G e t P r e f i x V a r ( C h a r t , " b X A x i s I s T i m e " )   t h e n  
                                                 G r a p h M i n X V a l   =   G r a p h M i n X V a l   /   6 0  
                                                 G r a p h M a x X V a l   =   G r a p h M a x X V a l   /   6 0  
                                         e n d   i f  
  
                                         i f   G e t P r e f i x V a r ( C h a r t , " b Y A x i s I s T i m e " )   t h e n  
                                                 G r a p h M i n Y V a l   =   G r a p h M i n Y V a l   /   6 0  
                                                 G r a p h M a x Y V a l   =   G r a p h M a x Y V a l   /   6 0  
                                         e n d   i f  
                                  
                                         C m d S t r   =   " S e t S c a l e   "   &   N d x   &   " , "   &   C S t r ( G r a p h M i n X V a l )   &   " , "   &   C S t r ( G r a p h M i n Y V a l )   &   " , "   &   C S t r ( G r a p h M a x X V a l )   &   " , "   &   C S t r ( G r a p h M a x Y V a l )  
                                         c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )  
                                 n e x t  
                         e n d   i f  
                  
                         f o r   N d x   =   1   t o   N u m T r a c e s  
                                 i f   G e t P r e f i x V a r ( C h a r t , " b H i d e G r a p h S y m b o l s " )   =   1   t h e n  
                                         c a l l   R u n T a g C m d ( C h a r t , " S h o w T r a c e S y m b o l   " & N d x & " , 0 " )  
                                 e l s e  
                                         c a l l   R u n T a g C m d ( C h a r t , " S h o w T r a c e S y m b o l   " & N d x & " , 1 " )                                                  
                                 e n d   i f  
                         n e x t  
                          
                         c a l l   R u n T a g C m d ( C h a r t ,   " Z o o m 1 0 0 "   )  
                         C m d S t r   =   " S e t Y A u t o S c a l e   "   &   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e Y " )  
                         c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )  
  
                         C m d S t r   =   " S e t X A u t o S c a l e   "   &   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e X " )  
                         c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )                          
                 e n d   i f  
         e n d   i f  
 e n d   s u b  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e G r a p h O p t i o n s E x t r a Y A x i s ( )  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   U p d a t e G r a p h O p t i o n s E x t r a Y A x i s ( C h a r t )  
         i f   G e t V a r ( C h a r t & " _ I n i t " )   =   1   t h e n    
                  
                 i f   R u n T a g C m d ( C h a r t , " I s C h a r t " )   =   " V a l i d C h a r t "   t h e n                  
                         c a l l   S e t V a r ( C h a r t & " _ I n i t " ,   0 )  
                         i f (   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e X " )   =   0   o r   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e Y " )   =   0   )   t h e n  
                                 G r a p h M i n X V a l   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M i n X V a l " )  
                                 G r a p h M a x X V a l   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M a x X V a l " )  
                                 G r a p h M i n Y V a l L e f t   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M i n Y V a l L e f t " )  
                                 G r a p h M a x Y V a l L e f t   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M a x Y V a l L e f t " )  
                                 G r a p h M i n Y V a l R i g h t   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M i n Y V a l R i g h t " )  
                                 G r a p h M a x Y V a l R i g h t   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M a x Y V a l R i g h t " )  
  
                                 i f   G e t P r e f i x V a r ( C h a r t , " b X A x i s I s T i m e " )   t h e n  
                                         G r a p h M i n X V a l   =   G r a p h M i n X V a l   /   6 0  
                                         G r a p h M a x X V a l   =   G r a p h M a x X V a l   /   6 0  
                                 e n d   i f  
  
                                 i f   G e t P r e f i x V a r ( C h a r t , " b Y A x i s I s T i m e L e f t " )   t h e n  
                                         G r a p h M i n Y V a l L e f t   =   G r a p h M i n Y V a l L e f t   /   6 0  
                                         G r a p h M a x Y V a l L e f t   =   G r a p h M a x Y V a l L e f t   /   6 0  
                                 e n d   i f  
  
                                 i f   G e t P r e f i x V a r ( C h a r t , " b Y A x i s I s T i m e R i g h t " )   t h e n  
                                         G r a p h M i n Y V a l R i g h t   =   G r a p h M i n Y V a l R i g h t   /   6 0  
                                         G r a p h M a x Y V a l R i g h t   =   G r a p h M a x Y V a l R i g h t   /   6 0  
                                 e n d   i f  
                          
                                 C m d S t r   =   " S e t S c a l e   1 , "   &   C S t r ( G r a p h M i n X V a l )   &   " , "   &   C S t r ( G r a p h M i n Y V a l L e f t )   &   " , "   &   C S t r ( G r a p h M a x X V a l )   &   " , "   &   C S t r ( G r a p h M a x Y V a l L e f t )  
                                 c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )  
  
                                 C m d S t r   =   " S e t S c a l e   2 , "   &   C S t r ( G r a p h M i n X V a l )   &   " , "   &   C S t r ( G r a p h M i n Y V a l R i g h t )   &   " , "   &   C S t r ( G r a p h M a x X V a l )   &   " , "   &   C S t r ( G r a p h M a x Y V a l R i g h t )  
                                 c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )                                  
                         e n d   i f  
                  
                         i f   G e t P r e f i x V a r ( C h a r t , " b H i d e G r a p h S y m b o l s " )   =   1   t h e n  
                                 c a l l   R u n T a g C m d ( C h a r t , " S h o w T r a c e S y m b o l   1 , 0 " )  
                         e l s e  
                                 c a l l   R u n T a g C m d ( C h a r t , " S h o w T r a c e S y m b o l   1 , 1 " )                                                  
                         e n d   i f  
                          
                         c a l l   R u n T a g C m d ( C h a r t ,   " Z o o m 1 0 0 "   )  
                         C m d S t r   =   " S e t Y A u t o S c a l e   "   &   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e Y " )  
                         c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )  
  
                         C m d S t r   =   " S e t X A u t o S c a l e   "   &   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e X " )  
                         c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )                          
                 e n d   i f  
         e n d   i f  
 e n d   s u b  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S e t T a g I f E m p t y ( )  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   S e t T a g I f E m p t y ( T a g , V a l u e )  
         v a l   =   G e t v a r ( T a g )  
         i f   i s E m p t y ( V a l )   t h e n   c a l l   S e t V a r ( T a g ,   V a l u e   )  
 e n d   s u b  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e P r e f i x C u r v e ( )  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   U p d a t e P r e f i x C u r v e ( P r e f i x )  
         P r e f i x   =   T r i m U n d e r s c o r e s ( P r e f i x )  
          
  
         i f   I s C h a n g e d (   P r e f i x     &   " _ _ T y p e " )   t h e n   C h a n g e d   =   t r u e  
         i f   I s C h a n g e d (   P r e f i x     &   " _ _ M a n u f a c t " )   t h e n   C h a n g e d   =   t r u e  
         i f   I s C h a n g e d (   P r e f i x     &   " _ _ M o d e l " )   t h e n   C h a n g e d   =   t r u e  
         i f   I s C h a n g e d (   P r e f i x     &   " _ _ C u r v e " )   t h e n   C h a n g e d   =   t r u e  
         i f   I s C h a n g e d (   P r e f i x     &   " _ _ T d m " )       t h e n   C h a n g e d   =   t r u e  
         c a l l   a l o g ( " U p d a t e   P r e f i x   C u r v e   "   &   P r e f i x   &   "   C h a n g e d   =   "   &   C h a n g e d   )  
  
         i f   C h a n g e d   t h e n    
                 T y p e S t r   =   G e t V a r (   P r e f i x   &   " _ _ T y p e " )  
                 M a n u f a c t   =   G e t V a r (   P r e f i x   &   " _ _ M a n u f a c t " )  
                 M o d e l   =   G e t V a r (   P r e f i x   &   " _ _ M o d e l " )  
                 N a m e   =   G e t V a r (   P r e f i x   &   " _ _ C u r v e " )  
                 k   =   G e t V a r ( P r e f i x   &   " _ _ T d m "   )  
 c a l l   a l o g ( " c u r v e   t y p e   =   "   &   T y p e S t r   )  
  
                 c a l l     L o a d C u r v e F r o m T a b l e (   P r e f i x   ,   T y p e S t r ,   M a n u f a c t ,   M o d e l ,     N a m e   ,   k   )  
         e n d   i f    
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   T r i m U n d e r s c o r e s ( )  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n   T r i m U n d e r s c o r e s ( T x t )  
         L e n g t h   =   L e n ( T x t )  
         i f   L e n g t h   a n d   r i g h t ( T x t , 1 )   =   " _ "   t h e n   T x t   =   l e f t ( T x t , L e n g t h - 1 )  
                  
         L e n g t h   =   L e n ( T x t )  
         i f   L e n g t h   a n d   r i g h t ( T x t , 1 )   =   " _ "   t h e n   T x t   =   l e f t ( T x t , L e n g t h - 1 )  
         T r i m U n d e r s c o r e s   =   T x t  
 e n d   f u n c t i o n    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   L o a d   a   c u r v e   f r o m   t h e   c u r v e   l i b r a r y  
 '   F i l l s   P r e f i x _ _ X _ # ,   P r e f i x _ _ Y _ #   v a r i a b l e s    
 '   A l s o   s e t s   P r e f i x _ _ N u m C u r v e P o i n t s   v a r i a b l e  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   L o a d C u r v e F r o m T a b l e (   P r e f i x   ,   T y p e S t r ,   M a n u f a c t ,   M o d e l ,     N a m e   ,   k   )  
         R e v   =   G e t I n t e r f a c e R e v ( )  
         c a l l   a L o g ( " R e v   =   "   &   R e v )  
         I f   R e v   <   1 7   T h e n  
                 c a l l   L o a d C u r v e F r o m T a b l e P r e R e v 1 7 ( P r e f i x ,   T y p e S t r ,   M a n u f a c t ,   M o d e l ,     N a m e   ,   k )  
         E l s e  
                 X S c a l e M u l t   =   G e t V a r ( " x S c a l e M u l t " )  
                 I f   I s B l a n k S t r i n g ( X S c a l e M u l t )   T h e n  
                         X S c a l e M u l t   =   1  
                 E n d   I f  
                  
                 '  
                 '   S e t   t h e   e q u a t i o n   t o   b l a n k   -   S e e   b u g z i l l a   6 0 4 0   f o r   d e t a i l s  
                 '  
                 c a l l   S e t V a r ( P r e f i x   &   " _ _ E q u a t i o n " ,   " "   )  
                  
                 c a l l   A p p l i c a t i o n . L o a d C u r v e ( F o r m . F o r m I d ,   T y p e S t r ,   M a n u f a c t ,   M o d e l ,   N a m e ,   k ,   P r e f i x ,   X S c a l e M u l t ,   " " )                  
         E n d   I f  
          
 e n d   s u b  
  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   O l d   V e r s i o n   o f   L o a d C u r v e F r o m T a b l e ,   o n l y   c a l l e d   i f   A u t o m a t i n   i n t e r f a c e   r e v i s i o n   i s   <   1 7  
 '  
 '   L o a d   a   c u r v e   f r o m   t h e   c u r v e   l i b r a r y  
 '   F i l l s   P r e f i x _ _ X _ # ,   P r e f i x _ _ Y _ #   v a r i a b l e s    
 '   A l s o   s e t s   P r e f i x _ _ N u m C u r v e P o i n t s   v a r i a b l e  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   L o a d C u r v e F r o m T a b l e P r e R e v 1 7 (   P r e f i x   ,   T y p e S t r ,   M a n u f a c t ,   M o d e l ,     N a m e   ,   k   )  
          
         x S c a l e M u l t   =   1  
         L e n g t h   =   l e n ( P r e f i x )  
         i f   L e n g t h   >   2   a n d     R i g h t (   P r e f i x , 2 )   =   " _ _ "   t h e n   P r e f i x   =   L e f t ( P r e f i x , l e n g t h - 2 )  
          
         w i t h   f o r m  
 '         S e l e c t S t r   =   " S E L E C T   X D a t a , Y D a t a , b U s e E q u a t i o n , S t a r t X , E n d X , S c a l e T y p e ,   E q u a t i o n ,   E r r o r P l u s ,   E r r o r M i n u s ,   E r r o r A b s P l u s , E r r o r A b s M i n u s   f r o m   C u r v e s   w h e r e   "   _  
 '                                   &     "   C u r v e N a m e   =   ' "   &   N a m e     &     " '   A N D   C u r v e T y p e   =   ' "   &   T y p e S t r   &   " '   A N D   M a n u f a c t u r e r   =   ' "   &   M a n u f a c t   &   " ' "   &   "   A N D   M o d e l   =   ' "   &   M o d e l   &   " ' "  
         S e l e c t S t r   =   " S E L E C T   *   f r o m   C u r v e s   w h e r e   "   _  
                                   &     "   C u r v e N a m e   =   ' "   &   N a m e     &     " '   A N D   C u r v e T y p e   =   ' "   &   T y p e S t r   &   " '   A N D   M a n u f a c t u r e r   =   ' "   &   M a n u f a c t   &   " ' "   &   "   A N D   M o d e l   =   ' "   &   M o d e l   &   " ' "  
  
         c a l l   S e t V a r ( P r e f i x   &   " _ _ N u m C u r v e P o i n t s " , 0 )  
                                    
       c a l l   a l o g ( " t d m =   "   &   k   &   "       s e l e c t   =   "   &   S e l e c t S t r   )  
         c a l l   S e t V a r ( " d b _ E r r o r M s g " , E m p t y )  
         c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S e l e c t S t r , 0 )  
          
         i f   . S q l E r r o r   =   E m p t y   t h e n    
  
                 C a l l   S e t V a r (   P r e f i x   &   " _ _ E r r o r P l u s " , . d b _ E r r o r P l u s )  
                 C a l l   S e t V a r (   P r e f i x   &   " _ _ E r r o r M i n u s " , . d b _ E r r o r M i n u s )  
                 C a l l   S e t V a r (   P r e f i x   &   " _ _ E r r o r A b s P l u s " , . d b _ E r r o r A b s P l u s )  
                 C a l l   S e t V a r (   P r e f i x   &   " _ _ E r r o r A b s M i n u s " , . d b _ E r r o r A b s M i n u s )  
                 C a l l   S e t V a r (   P r e f i x   &   " _ _ E q u a t i o n " , . d b _ E q u a t i o n )  
                 C a l l   S e t V a r (   P r e f i x   &   " _ _ R e s e t E q u a t i o n " ,   G e t V a r ( " d b _ R e s e t E q u a t i o n " )   )  
                  
                 '   c a l l   a l o g ( " S e t t i n g   e q u a t i o n   ' "   &     P r e f i x   &   " _ _ E q u a t i o n '   =   "   &   . d b _ E q u a t i o n   )  
                  
                 i f   . d b _ b U s e E q u a t i o n   t h e n    
                         S t L o g L o g   =   4  
                         S t S e m i L o g X   =   2  
                         S c a l e T y p e   =   T o N u m b e r (   . d b _ S c a l e T y p e   )  
                         S t a r t X   =   . d b _ S t a r t X  
                         i f   S t a r t X   =   1   t h e n   S t a r t X   =   1 . 0 1  
                         '  
                         '   I f   l o g - l o g   o r   s e m i - l o g   X    
                         '    
                         i f   S c a l e T y p e   =   S t L o g L o g       o r   S c a l e T y p e   =   S t S e m i L o g X     t h e n    
                                 c a l l   C a l c C u r v e L o g X ( P r e f i x , S t a r t X ,   . d b _ E n d X ,   1 0 0 ,   k ,   "   y   =   "     &   . d b _ E q u a t i o n   )  
                         e l s e    
                                 c a l l   C a l c C u r v e L i n e a r ( P r e f i x , S t a r t X ,   . d b _ E n d X ,   1 0 0 ,   k ,   "   y   =   "     &   . d b _ E q u a t i o n   )  
                         e n d   i f    
                          
                 e l s e    
                         i f   . d b _ E q u a t i o n   =   " a b s "   t h e n   x S c a l e M u l t   =   G e t V a r ( " x S c a l e M u l t " )  
                  
                         f o r   i   =   1   t o   1 0 0    
                                 c a l l   S e t N d x V a r ( P r e f i x   &   " _ _ x _ "   &   N d x , i ,   E m p t y   )  
                                 c a l l   S e t N d x V a r ( P r e f i x   &   " _ _ y _ "   &   N d x , i ,   E m p t y   )  
                         n e x t  
                  
                         i f     . d b _ E r r o r M s g   < >   " "   t h e n    
                                 c a l l   a l o g ( " D B   E R R O R :   "   &   . d b _ E r r o r M s g   )  
                         e l s e i f   . d b _ N u m R e c o r d s   >   0   t h e n    
                                 c a l l   a l o g ( " x d a t a   =   "   &   . d b _ x D a t a   )  
                                 c a l l   a l o g ( " y d a t a   =   "   &   . d b _ y D a t a   )  
                                 d i m   X D a t a ( 2 0 0 )  
                                 d i m   Y D a t a ( 2 0 0 )  
                                 c a l l   P a r s e S t r i n g (   . d b _ X D a t a ,   X D a t a , " | " , 2 0 0 )  
                                 c a l l   P a r s e S t r i n g (   . d b _ Y D a t a ,   Y D a t a , " | " , 2 0 0 )  
                                  
                                                                  
                                 '  
                                 '   x S c a l e   M u l t  
                                 '    
                                 i f   x S c a l e e M u l t   =   E m p t y   t h e n   x S c a l e M u l t   =   1  
                                 '  
                                 '   C h e c k   f o r   t i m e   d i a l   d a t a  
                                 '    
                                 b T i m e D i a l D a t a   =   f a l s e    
                                 f o r   i   =   2   t o   2 0 0    
  
                                         i f   I s E m p t y (   X D a t a ( i )   )   t h e n   e x i t   f o r  
  
                                         x D a t a N   =   t o N u m b e r ( X D a t a ( i ) )  
                                         x D a t a N M i n u s 1   =   T o N u m b e r ( x d a t a ( i - 1 ) )  
  
  
                                       i f   x D a t a N   <   x D a t a N M i n u s 1   t h e n    
                                               b T i m e D i a l D a t a   =   t r u e  
                                               e x i t   f o r    
                                       e n d   i f    
                                 n e x t    
                                 c a l l   a l o g ( " T i m e D i a l D a t a   =   "   &   b T i m e D i a l D a t a   )  
                                 T d m   =   T o N u m b e r (   k   )  
                                 i f   b T i m e D i a l D a t a   a n d   T d m   < >   E m p t y   a n d   T d m   > =   . 5   a n d   T d m   <   1 2   t h e n    
                                         T d D a t a   =   a r r a y (   . 5 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 1 0 , 1 1 , 1 2 , 1 3 , 1 4 , 1 5 , 1 6 , 1 7 )  
                                         T d N d x   =   0  
                                         d i m   Y T d ( 2 0 , 1 0 0 )  
                                         P t N d x   =   1  
                                         f o r   i   =   1   t o   2 0 0    
  
                                                 x D a t a N   =   t o N u m b e r ( X D a t a ( i ) )  
                                                 x D a t a N M i n u s 1   =   T o N u m b e r ( x d a t a ( i - 1 ) )  
  
                                           '       c a l l   a l o g ( " X D a t a ( "   &   i   &   "   )   =   "   &   x D a t a N   &   " , "   &   x D a t a N M i n u s 1   )  
                                                 i f   I s E m p t y (   X D a t a ( i )   )   t h e n   e x i t   f o r  
                                                  
                                               i f   i   >   1   a n d     x D a t a N   <   x D a t a N M i n u s 1   t h e n    
                                                       T d N d x   =   T d N d x   +   1    
                                                       P t N d x   =   1  
                                             '           c a l l   a l o g ( " N e w   T i m e   D i a l "   &   x D a t a ( i )   &   " , "   &   x D a t a ( i - 1 )   ) '  
  
                                                 e n d   i f    
                                                 Y T d ( T d N d x ,   P t N d x   )   =   y D a t a ( i )  
                                                 p t N d x   =   P t N d x   +   1  
                                                 c a l l   a l o g (   " Y d a t a ( "   &   T d N d x   &   " , "   &   P t N d x   &   "   )   =   "   &   y D a t a ( i ) )  
  
                                         n e x t    
                                          
                                         f o r   i   =   0   t o   1 2    
                                               c a l l   a l o g ( " T d m   =   "   &   T d m   &   "       T d D a t a ( "   &   i   &   " )   =   "   &   T d D a t a ( i ) )  
                                               i f   T d m   < =   T d D a t a ( i )   t h e n    
                                                     N d x 1   =   i   -   1  
                                                     N d x 2   =   i    
                                                     i f   T d m   =   T d D a t a ( i )   t h e n   N d x 1   =   i  
                                                     e x i t   f o r    
                                             e n d   i f    
                                         n e x t  
                                         T d m 1   =   T d D a t a (   N d x 1 )  
                                         T d m 2   =   T d D a t a (   N d x 2 )  
                                         P e r c e n t O f 2   =   T d m   -   T d m 1    
                                         i f   P e r c e n t O f 2   t h e n   P e r c e n t O f 2   =   D i v i d e (   P e r c e n t O f 2   ,   ( T d m 2   -   T d m 1 ) )  
                                         c a l l   a l o g ( " P o f 2   =   "   &   P e r c e n t O f 2   &   "       "   &   N d x 1   &   " , "   &   N d x 2   &   "         T d m 1 2   =   "   &   T d m 1   &   " , "   &   T d m 2 )  
                                          
                                         f o r   i   =   1   t o   2 0    
                                             Y   =   Y t d (   T d m 1 , i )  
                                             i f   I s E m p t y ( Y )   t h e n    
                                                     Y D a t a ( i )   =   Y  
                                                     X D a t a ( i )   =   E m p t y    
                                                     e x i t   f o r    
                                             e n d   i f    
                                             i f   P e r c e n t O f 2   t h e n   Y   =   Y   +   ( (   Y t d ( T d m 2 , i )   -   Y   )   *   P e r c e n t O f 2   )  
                                             c a l l   a l o g (   " Y ( "   &   i   &   " )   =   "   &   Y   )  
                                               y D a t a ( i )   =   Y  
                                         n e x t    
                                 e n d   i f    
                                 b d o n e   =   f a l s e  
                                 f o r   i   =   1   t o   2 0 0    
                                         i f   I s E m p t y (   X D a t a ( i )   )   t h e n     b D o n e   =   t r u e    
                                       i f   b D o n e   t h e n    
                                                 c a l l   S e t N d x V a r ( P r e f i x   &   " _ _ x "   &   N d x , i ,   E m p t y )  
                                                 c a l l   S e t N d x V a r ( P r e f i x   &   " _ _ y "   &   N d x , i ,   E m p t y )  
                                         e l s e  
                  
                                                 c a l l   S e t N d x V a r ( P r e f i x   &   " _ _ x "   &   N d x , i ,   X D a t a ( i )   *   x S c a l e M u l t     )  
                                                 c a l l   S e t N d x V a r ( P r e f i x   &   " _ _ y "   &   N d x , i ,   y D a t a ( i )   )  
                                                 c a l l   a l o g (   i   &   " , "   &   x D a t a ( i )   &   " , "   &   y D a t a ( i )   )  
                                                 c a l l   S e t V a r ( P r e f i x   &   " _ _ N u m C u r v e P o i n t s " , i )  
                                         e n d   i f    
                                 n e x t  
                         e n d   i f    
                 e n d   i f  
         e n d   i f    
         e n d   w i t h    
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   E v a l u a t e   a   p o i n t   o n   a   c u r v e   t h a t   i s   a l r e a d y   l o a d e d    
 '  
 '   K e p t   f o r   b a c k w a r d s   c o m p a t i b i l i t y ,   c a l l s   E v a l C u r v e E x t r a P a r a m s ( )  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n   E v a l C u r v e (   C u r v e ,   M u l t ,   T d m   )  
          
         E v a l C u r v e   =   E v a l C u r v e E x t r a P a r a m s (   C u r v e ,   M u l t ,   T d m ,   " "   )  
          
 e n d   f u n c t i o n  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   E v a l u a t e   a   p o i n t   o n   a   c u r v e   t h a t   i s   a l r e a d y   l o a d e d    
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n   E v a l C u r v e E x t r a P a r a m s (   C u r v e ,   M u l t ,   T d m ,   E x t r a P a r a m s   )  
         E q u a t i o n   =   G e t V a r (   C u r v e   &   " E q u a t i o n " )  
          
         '   s e t   d e f a u l t s   f o r   p i c k u p / t a p  
         p i c k u p   =   1  
         t a p   =   1  
  
         ' o v e r r i d e   w i t h   E x t r a P a r a m s  
         i f   N o t   I s B l a n k S t r i n g ( E x t r a P a r a m s )   t h e n  
               o n   e r r o r   r e s u m e   n e x t  
               C o m m a n d L i s t   =   R e p l a c e ( E x t r a P a r a m s ,   " , " ,   " : " )  
               c a l l   E x e c u t e ( C o m m a n d L i s t )  
               o n   e r r o r   g o t o   0    
         e n d   i f  
  
         k   =   T d m  
          
            
         x   =   M u l t  
         M   =   M u l t  
         I   =   M u l t  
         y   =   E m p t y  
         o n   e r r o r   r e s u m e   n e x t  
         c a l l   E x e c u t e (   "   y   =   "   &   E q u a t i o n   )  
         o n   e r r o r   g o t o   0    
  
   c a l l   a l o g ( " E v a l C u r v e (   "   &   C u r v e   &   " , "   &   M   &   " , "   &   T d m   &   " )   =   "   &   y   &   "             E q   =   "   &   E q u a t i o n   )  
         '  
         '   I f   n o   e q u a t i o n ,   t r y   a   c h a r t   l o o k u p  
         '  
         i f   I s E m p t y ( y )   t h e n    
                 C h a r t   =   G e t V a r ( C u r v e   &   " C h a r t " )  
                 T r a c e   =   G e t V a r ( C u r v e   &   " T r a c e " )  
                 c a l l   a l o g ( " E v a l C u r v e ( " & C u r v e & " )   "   &   C h a r t   &   " , "   &   T r a c e   )  
                 i f   C h a r t   < >   E m p t y   a n d   T r a c e   < >   E m p t y   t h e n   y   =     R u n T a g C m d ( C h a r t , " L o o k u p Y   "   &   T r a c e   &   "   "   &   M u l t   )  
         e n d   i f    
  
         E v a l C u r v e E x t r a P a r a m s   =     y    
 e n d   f u n c t i o n    
  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S w e e p   t h r o u g h   a   u s e r   d e f i n e d   e q u a t i o n   w i t h   a   l o g   x   s c a l e  
 '  
 '   K e p t   f o r   b a c k w a r d s   c o m p a t i b i l i t y ,   c a l l s   C a l c C u r v e L o g X E x t r a P a r a m s ( )  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   C a l c C u r v e L o g X ( P r e f i x , S t a r t V a l , E n d V a l , N u m P o i n t s , k , E q u a t i o n )  
          
         C a l l   C a l c C u r v e L o g X E x t r a P a r a m s ( P r e f i x , S t a r t V a l , E n d V a l , N u m P o i n t s , k , E q u a t i o n , " " )  
          
 e n d   s u b  
          
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S w e e p   t h r o u g h   a   u s e r   d e f i n e d   e q u a t i o n   w i t h   a   l o g   x   s c a l e    
 '  
 '   E x t r a P a r a m s :   V a l u e s   o f   e x t r a   p a r a m e t e r s   t o   b e   m a d e   a v a i l a b l e   t o  
 '                             e q u a t i o n .   S h o u l d   b e   c o l o n   ( o r   c o m m a )   s e p a r a t e d  
 '  
 '                             e g :   " p i c k u p = 4 . 3 , x x = 1 2 " .  
 '  
 '                             N o t e :   O n l y   p i c k u p   a n d   t a p   a r e   s u p p o r t e d   f o r   n o w  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   C a l c C u r v e L o g X E x t r a P a r a m s ( P r e f i x , S t a r t V a l , E n d V a l , N u m P o i n t s , k , E q u a t i o n , E x t r a P a r a m s )  
   w i t h   F o r m  
       i f   I s N u m e r i c ( E n d V a l )   a n d   n o t   I s E m p t y ( E n d V a l )   a n d   I s N u m e r i c ( S t a r t V a l )   a n d   n o t   I s E m p t y ( S t a r t V a l )   t h e n  
  
           '   A v o i d   e r r o r s   w h e n   c a l c u l a t i n g   l o g  
           I f   E n d V a l   < =   0   T h e n  
               E n d V a l   =   1  
           E n d   I f  
           I f   S t a r t V a l   < =   0   T h e n  
               S t a r t V a l   =   0 . 0 0 1  
           E n d   I f  
  
           L o g 1 0 E n d   =   L o g 1 0 ( E n d V a l )  
           L o g 1 0 S t a r t   =   L o g 1 0 ( S t a r t V a l )  
  
           D e l t a L o g 1 0   =   L o g 1 0 E n d   -   L o g 1 0 S t a r t    
           I n c L o g 1 0   =   D e l t a L o g 1 0   /   ( N u m P o i n t s   -   1 )  
           L o g V a l   =   L o g 1 0 S t a r t  
              
           '   s e t   d e f a u l t s   f o r   p i c k u p / t a p  
           p i c k u p   =   1  
           t a p   =   1  
  
           ' o v e r r i d e   w i t h   E x t r a P a r a m s  
           I f   N o t   I s B l a n k S t r i n g ( E x t r a P a r a m s )   T h e n  
               o n   e r r o r   r e s u m e   n e x t  
               C o m m a n d L i s t   =   R e p l a c e ( E x t r a P a r a m s ,   " , " ,   " : " )  
               c a l l   E x e c u t e ( C o m m a n d L i s t )  
               o n   e r r o r   g o t o   0    
           E n d   I f  
  
           T d m   =   k  
  
           N d x   =   0  
           f o r   P o i n t   =   1   t o   N u m P o i n t s  
              
               x   =   1 0   ^   L o g V a l                
                      
               M   =   x  
               I   =   x  
               y   =   E m p t y  
               o n   e r r o r   r e s u m e   n e x t  
               c a l l   E x e c u t e (   E q u a t i o n   )  
               o n   e r r o r   g o t o   0    
  
               ' c a l l   a l o g (   " p o i n t   "   &   P o i n t   &   " f ( "   &   x   & " )   =   "   &   y   )  
               i f   N o t   I s E m p t y ( y )   t h e n    
                   N d x   =   N d x   +   1  
                              
                   c a l l   S e t N d x V a r ( P r e f i x +   " _ _ Y " , N d x , y )  
                   c a l l   S e t N d x V a r ( P r e f i x   +   " _ _ X " , N d x , x )  
                              
                   '       c a l l   a l o g (   P r e f i x   &   " _ _ X         X Y   =   "   &   i   &   " , "   &   x   &   " , "   &   y   )  
               e n d   i f    
               L o g V a l   =   L o g V a l   +   I n c L o g 1 0    
                      
           n e x t  
       e n d   i f  
       c a l l   S e t V a r ( P r e f i x   +   " _ _ N u m C u r v e P o i n t s " , N d x     )  
   e n d   w i t h    
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S w e e p   t h r o u g h   a   u s e r   d e f i n e d   e q u a t i o n   w i t h   a   l i n e a r   x   s c a l e    
 '  
 '   K e p t   f o r   b a c k w a r d s   c o m p a t i b i l i t y ,   c a l l s   C a l c C u r v e L i n e a r E x t r a P a r a m s ( )  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   C a l c C u r v e L i n e a r ( P r e f i x , S t a r t V a l , E n d V a l , N u m P o i n t s , k , E q u a t i o n )  
          
         C a l l   C a l c C u r v e L i n e a r E x t r a P a r a m s ( P r e f i x , S t a r t V a l , E n d V a l , N u m P o i n t s , k , E q u a t i o n , " " )  
 e n d   s u b  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S w e e p   t h r o u g h   a   u s e r   d e f i n e d   e q u a t i o n   w i t h   a   l i n e a r   x   s c a l e    
 '  
 '   E x t r a P a r a m s :   V a l u e s   o f   e x t r a   p a r a m e t e r s   t o   b e   m a d e   a v a i l a b l e   t o  
 '                             e q u a t i o n .   S h o u l d   b e   c o l o n   ( o r   c o m m a )   s e p a r a t e d  
 '  
 '                             e g :   " p i c k u p = 4 . 3 , x x = 1 2 " .  
 '  
 '                             N o t e :   O n l y   p i c k u p   a n d   t a p   a r e   s u p p o r t e d   f o r   n o w  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   C a l c C u r v e L i n e a r E x t r a P a r a m s ( P r e f i x , S t a r t V a l , E n d V a l , N u m P o i n t s , k , E q u a t i o n , E x t r a P a r a m s )  
     w i t h   F o r m  
         i f   I s N u m e r i c ( E n d V a l )   a n d   n o t   I s E m p t y ( E n d V a l )   a n d   I s N u m e r i c ( S t a r t V a l )   a n d   n o t   I s E m p t y ( S t a r t V a l )   t h e n  
             D e l t a     =   E n d V a l   -   S t a r t V a l    
             x   =   S t a r t V a l  
             I n c r e m e n t   =   D e l t a   /   N u m P o i n t s  
  
             '   s e t   d e f a u l t s   f o r   p i c k u p / t a p  
             p i c k u p   =   1  
             t a p   =   1  
  
             ' o v e r r i d e   w i t h   E x t r a P a r a m s  
             I f   N o t   I s B l a n k S t r i n g ( E x t r a P a r a m s )   T h e n  
                 o n   e r r o r   r e s u m e   n e x t  
                 C o m m a n d L i s t   =   R e p l a c e ( E x t r a P a r a m s ,   " , " ,   " : " )  
                 c a l l   E x e c u t e ( C o m m a n d L i s t )  
                 o n   e r r o r   g o t o   0    
             E n d   I f  
  
             f o r   P o i n t   =   1   t o   N u m P o i n t s  
  
                 c a l l   S e t N d x V a r ( P r e f i x   +   " _ _ X " , P o i n t , x )  
                  
                 M   =   x  
                 I   =   x  
                 y   =   E m p t y  
                 o n   e r r o r   r e s u m e   n e x t                  
                 c a l l   E x e c u t e (   E q u a t i o n   )  
                 o n   e r r o r   g o t o   0                    
                 c a l l   S e t N d x V a r ( P r e f i x +   " _ _ Y " , P o i n t , y )  
                 x   =   x   +   I n c r e m e n t    
                  
             n e x t  
         e n d   i f  
  
         c a l l   S e t V a r ( P r e f i x   +   " _ _ N u m C u r v e P o i n t s " , N u m P o i n t s )  
     e n d   w i t h    
 e n d   s u b    
  
  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t I n t e r f a c e R e v ( )  
 '   R e t u r n s   t h e   v e r s i o n   o f   t h e   a u t o m a t i o n   i n t e r f a c e  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   G e t I n t e r f a c e R e v ( )  
         G e t I n t e r f a c e R e v   =   0  
         o n   e r r o r   r e s u m e   n e x t    
         G e t I n t e r f a c e R e v   =     A p p l i c a t i o n . G e t I n t e r f a c e R e v ( )  
         o n   e r r o r   g o t o   0  
 e n d   f u n c t i o n  
  
 '  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   C t o F ( )  
 '   C o n v e r t   d e g r e e s   C   t o   d e g r e e s   F  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   C t o F (   D e g C )  
         C t o F   =   E m p t y  
         i f   n o t   I s E m p t y (   D e g C   )   t h e n   C t o F   =         T o N u m b e r ( D e g C )   *   9 . 0   /   5 . 0     +   3 2 . 0  
 e n d   f u n c t i o n  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   F t o C ( )  
 '   C o n v e r t   d e g r e e s   F   t o   d e g r e e s   C  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   F t o C (   D e g F )  
         F t o C   =   E m p t y  
         i f   n o t   I s E m p t y (   D e g F   )   t h e n   F t o C   =     ( T o N u m b e r ( D e g F )   -   3 2 )   *   5 . 0   /   9 . 0      
 e n d   f u n c t i o n  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   H H M M S S t o S e c s ( )  
 '   C o n v e r t   a   H H : M M : S S   s t r i n g   t o   s e c o n d s .  
 '   a l s o   s u p p o r t s   M M : S S   a n d   S S  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
  
 f u n c t i o n   H H M M S S t o S e c s ( T i m e S t r )  
         d i m   p a r t s ( 3 )  
         V a l u e   =   0  
         c o u n t   =   P a r s e S t r i n g ( T i m e S t r ,   p a r t s ,   " : " ,   3 )  
         f o r   i   =   1   t o   c o u n t  
                 V a l u e   =   V a l u e   *   6 0  
                 V a l u e   =   V a l u e   +     T o N u m b e r (   p a r t s ( i )   )  
         n e x t  
         h h M M S S t o S e c s   =   V a l u e  
 e n d   f u n c t i o n  
          
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   I s C h a n g e d ( )  
 '   R e t u r n   t r u e   i f   t h e   v a r i a b l e   h a s   c h a n g e d   s i n c e   t h e   l a s t   c a l l    
 '   U s e   U n i q u e   k e y   f o r   m u l t i p l e   i s   c h a n g e d   f o r   s a m e   v a r i a b l e  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n   I s C h a n g e d ( V a r N a m e )  
             I s C h a n g e d   =   I s C h a n g e d E x ( V a r N a m e , " 1 " )  
 e n d   f u n c t i o n    
  
 f u n c t i o n   I s C h a n g e d E x ( V a r N a m e , K e y )  
         I s C h a n g e d E x   =   f a l s e  
         C h a n g e V a r   =   " _ _ C h a n g e d _ _ "   +   V a r N a m e   +   K e y  
         L a s t V a l u e   =   G e t V a r ( C h a n g e V a r )  
         T h i s V a l u e   =   G e t V a r ( V a r N a m e )  
  
          
          
         i f   I s E m p t y ( L a s t V a l u e )   t h e n    
                 I s C h a n g e d E x   =   t r u e  
         e l s e  
                 i f   L a s t V a l u e   =   " _ _ E m p t y _ _ "   t h e n    
                         i f   n o t   I s E m p t y ( T h i s V a l u e )   t h e n   I s C h a n g e d E x   =   t r u e  
                 e l s e    
                         i f   L a s t V a l u e   < >   T h i s V a l u e   t h e n   I s C h a n g e d E x   =   t r u e    
                 e n d   i f    
         e n d   i f    
         i f   I s C h a n g e d E x   t h e n    
                  
                 ' c a l l   a l o g (   " C h a n g e d :     "   +   C h a n g e V a r   +   "       "   +   C S t r ( L a s t V a l u e )   +   "         "   +   C S t r ( T h i s V a l u e )   )          
                  
                 i f   I s E m p t y ( T h i s V a l u e )   t h e n    
                         c a l l   S e t V a r ( C h a n g e V a r , " _ _ E m p t y _ _ " )  
                 e l s e    
                         c a l l   S e t V a r (   C h a n g e V a r , T h i s V a l u e )  
                 e n d   i f    
         e n d   i f    
 e n d   f u n c t i o n    
  
  
  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t V a r T c f (   B y V a l   D e g C ,   B y V a l   T e m p V a r N a m e ,   B y V a l   T a b l e N a m e ,   B y V a l   K C o l u m n N a m e   )  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 ' 	  
 	 	 	 	 	 	 	 	     '  
 F U N C T I O N   G e t V a r T c f (   B y V a l   D e g C ,   B y V a l   T e m p V a r N a m e ,   B y V a l   T a b l e N a m e ,   B y V a l   K C o l u m n N a m e   )  
  
 '   N O T E   T h a t   t h i s   f u n c t i o n   c a n   n o t   b e   u s e d   o n   t a b l e s   w i t h   i n t e g e r   t e m p e r a t u r e s  
  
         S e l e c t S t a t e m e n t H i g h   =   " S E L E C T   T O P   1   "   +   T e m p V a r N a m e   +   "   a s   [ T e m p C ]   ,   "   +   K C o l u m n N a m e   +   "   a s   [ K F a c t o r ]   F R O M   "   +   T a b l e N a m e   +   _  
                 "   W H E R E   "   +   T e m p V a r N a m e   +   "     > =   "   +   C s t r ( D e g C )   +   _  
                 "   O R D E R   B Y   "   +   T e m p V a r N a m e    
  
         S e l e c t S t a t e m e n t L o w   =   " S E L E C T   T O P   1   "   +   T e m p V a r N a m e   +   "   a s   [ T e m p C ]     ,   "   +   K C o l u m n N a m e   +   "   a s   [ K F a c t o r ]   F R O M   "   +   T a b l e N a m e   +   _  
                 "   W H E R E   "   +   T e m p V a r N a m e   +   "     < =   "   +   C s t r ( D e g C )   +   _  
                 "   O R D E R   B Y   "   +   T e m p V a r N a m e   +   "   D E S C "  
  
  
         K F a c t o r   =   e m p t y  
  
  
         '  
         '   I f   a   v a l i d   t e m p e r a t u r e   t o   l o o k u p  
         '  
         I F   N O T   I s E m p t y ( D e g C )   A N D   I s N u m e r i c ( D e g C )   T H E N  
  
                 '  
                 '   G e t   t h e   p o i n t   a b o v e   t h e   l o o k u p   p o i n t  
                 '  
                 F o r m . D e c l a r e V a r ( " d b _ T e m p C " )  
                 F o r m . D e c l a r e V a r ( " d b _ K F a c t o r " )  
                 c a l l   A p p l i c a t i o n . Q u e r y L o o k u p T a b l e ( F o r m . F o r m I d ,   S e l e c t S t a t e m e n t H i g h ,   0 )  
                 T e m p H i g h   =   F o r m . d b _ T e m p C  
                 K H i g h         =   F o r m . d b _ K F a c t o r  
  
                  
                 '  
                 '   G e t   t h e   f a c t o r   b e l o w   t h e   l o o k u p   p o i n t  
                 '  
                 c a l l   A p p l i c a t i o n . Q u e r y L o o k u p T a b l e ( F o r m . F o r m I d ,   S e l e c t S t a t e m e n t L o w ,   0 )  
                 T e m p L o w   =   F o r m . d b _ T e m p C  
                 K L o w         =   F o r m . d b _ K F a c t o r  
                  
                  
  
                 c a l l   a l o g (   C S t r ( T e m p L o w )   +   " , "   +   C S t r ( K L o w )   +   " , "   +   C S t r ( T e m p H i g h )   +   " , "   +   C S t r ( K H i g h ) )  
  
  
                 '  
                 '   I f   t h e   l o o k u p   p o i n t   w a s   w i t h i n   t h e   t a b l e ,   i n t e r p o l a t e   t h e   r e s u l t s  
                 '  
                 i f   T e m p L o w   < >   e m p t y   a n d   T e m p H i g h   < >   e m p t y   t h e n  
                         D e l t a T   =   T e m p H i g h   -   T e m p L o w  
                         D e l t a K   =   K H i g h   -   K L o w  
                         i f   D e l t a T   >   0   t h e n  
                                 R a t i o   =   ( D e g C   -   T e m p L o w )   /   D e l t a T  
                                 K F a c t o r   =   K L o w   +   (   D e l t a K   *   R a t i o   )  
                         e l s e  
                                 K F a c t o r   =   K L o w  
                         e n d   i f  
  
 	 	 	   F o r m . D e c l a r e V a r ( " P d b T e m p C o r r F a c t o r " )  
  
                         '  
                         '   c o n v e r t   f r o m   v t _ r 4   t o   s t r i n g ,   t o   v t _ r 8  
                         '   t o   a v o i d   c o e r s i o n   r e s o l u t i o n   p r o b l e m .  
                         '  
                         S t r T c f   =   C S t r ( K F a c t o r )  
                         K F a c t o r   =   C D b l ( S t r T c f )  
                 E N D   I F  
  
         E N D   I F  
         G e t V a r T c f   =   K F a c t o r  
 E N D   F U N C T I O N  
  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   A d d S a v e d V a r ( )  
 '  
 '   W r a p p e r   a r o u n d   t h e   a p p l i c a t i o n   c a l l   t o   i g n o r e   f o r   e a r l y   r e v s  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   A d d S a v e d V a r (     V a r N a m e   )  
         o n   e r r o r   r e s u m e   n e x t    
         c a l l   A p p l i c a t i o n . A d d S a v e d V a r ( F o r m . F o r m I d , V a r N a m e )  
         o n   e r r o r   g o t o   0  
 e n d   s u b    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e A s F o u n d A s L e f t ( N a m e )  
 '  
 '   S e t s   t h e   A l < N a m e >   v a r i a b l e   t o   t h e   A f < N a m e >   v a r i a b l e   i f   i t   i s    
 '   e m p t y    
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   U p d a t e A s F o u n d A s L e f t ( N a m e )  
         A f N a m e   =   " A f "   +   N a m e    
         A l N a m e   =   " A l "   +   N a m e    
         c a l l   A p p l i c a t i o n . U p d a t e A s F o u n d A s L e f t (   F o r m . F o r m I d ,   A f N a m e , A l N a m e )  
 e n d   S u b  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e A s F o u n d A s L e f t N d x ( N a m e , N d x )  
 '  
 '   S e t s   t h e   A l < N a m e > _ < n d x >   v a r i a b l e   t o   t h e   A f < N a m e > < n d x >   v a r i a b l e   i f   i t   i s    
 '   e m p t y    
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   U p d a t e A s F o u n d A s L e f t N d x ( N a m e ,   N d x )  
         c a l l   U p d a t e A s F o u n d A s L e f t (   N a m e   +   " _ "   +   C S t r ( N d x )   )  
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S e t N u m R o w s ( )  
 '  
 '   S e t s   t h e   n u m b e r   o f   r o w s   f o r   t h e   s p e c i f i e d   t e m p l a t e   i f   i t   h a s    
 '   c h a n g e d    
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   S e t N u m R o w s (   T e m p l a t e N a m e ,   N u m R o w s   )  
                 c a l l   A p p l i c a t i o n . R u n T a g C m d   (   F o r m . F o r m I d ,   T e m p l a t e N a m e ,   " S e t N u m R o w s   "   +   C S t r ( N u m R o w s )   )  
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S e t V i s i b l e ( )  
 '   S h o w s   o r   h i d e s   a   t e m p l a t e   o r   f i e l d    
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   S e t V i s i b l e ( N a m e ,   V i s i b l e )  
   C m d   =   " v i s i b l e   0 "  
   F a l s e S t r   =   C S t r ( f a l s e )  
   i f   n o t   I s E m p t y ( V i s i b l e )   t h e n  
       V i s S t r   =   C S t r ( V i s i b l e )  
       i f   V i s S t r   < >   F a l s e S t r   a n d   V i s S t r   < >   " 0 "   a n d   V i s S t r   < >   " - 2 "   t h e n  
           C m d   =   " v i s i b l e   1 "  
       e n d   i f  
   e n d   i f  
   c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   N a m e ,   C m d )  
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t L i s t ( )  
 '   R e a d s   a   l i s t   o f   v a l u e s   f r o m   t h e   d a t a   t a b   a n d   r e t u r n s   i t  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n   G e t L i s t ( t a g )  
         G e t L i s t   =   a p p l i c a t i o n . R e a d D a t a M u l t i L i n e F r o m T a g ( F o r m . F o r m I d ,   T a g )  
 e n d   f u n c t i o n    
  
 ' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
 '   P a r s e s   t h e   v a l u e   r e t u r n e d   b y   a p p l i c a t i o n . g e t V e r s i o n  
 '   a n d   c o n s t r u c t s   t h e   v e r s i o n   n u m b e r .  
 '   e g :   I f   g e t V e r s i o n   r e t u r n s   " 4 . 1 2 . T R U N K . 2  
 '   t h e   v e r s i o n   n u m b e r   i s :   4   *   1 0 0 0   +   1 2   +   0 . 2   =   4 0 1 2 . 2  
 ' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
  
 f u n c t i o n   g e t V e r s i o n  
  
         o n   e r r o r   r e s u m e   n e x t  
         ' e n a b l e   e r r o r   h a n d l i n g :   I n   c a s e   a p p l i c a t i o n . g e t V e r s i o n   i s   n o t   d e f i n e d .  
         v e r s i o n S t r   =   a p p l i c a t i o n . g e t V e r s i o n  
         ' d i s a b l e   e r r o r   h a n d l i n g  
         o n   e r r o r   g o t o   0  
  
         i f   I s E m p t y ( v e r s i o n S t r )   t h e n  
                 g e t V e r s i o n   =   E m p t y  
         e l s e  
  
                 '   C h e c k   f o r   a t   m o s t   5   d o t   ( . )   s e p a r a t e d   p a r t s   i n   v e r s i o n   s t r i n g .  
                 '   N O T E :   P a r s e S t r i n g ( )   d o e s n ' t   u s e   t h e   0 ' t h   e l e m e n t   o f   t h e   a r r a y .  
                 '               T h e r e f o r e ,   w e   c r e a t e   a n   a r r a y   o f   6   e l e m e n t s  
                 D i m   p a r t s ( 6 )  
                 c o u n t   =   P a r s e S t r i n g ( v e r s i o n S t r ,   p a r t s ,   " . " ,   5 )  
  
                 '   G o   t h r o u g h   t h e   p a r s e d   v a l u e s   a n d   g e t   t h e   f i r s t   3   n u m e r i c   v a l u e s .  
                 D i m   n  
                 D i m   v e r s i o n N u m b e r  
                 n   =   0  
  
                 f o r   i   =   1   t o   c o u n t  
                         s t r   =   p a r t s ( i )  
  
                         i f   I s N u m e r i c ( s t r )   t h e n  
                                 n   =   n   +   1  
  
                                 '   I f   t h e   f i r s t   3   n u m b e r s   a r e   4 , 1 2 , 5 ,   T h e   v e r s i o n   w i l l   b e   4 0 1 2 . 5  
                                 s e l e c t   c a s e   n  
                                         c a s e   1  
                                                 v e r s i o n N u m b e r   =   1 0 0 0   *   C I n t ( s t r )  
                                         c a s e   2  
                                                 v e r s i o n N u m b e r   =   v e r s i o n N u m b e r   +   C I n t ( s t r )  
                                         c a s e   3  
                                                  
                                                 ' T h e   t h i r d   n u m b e r   i s   t h e   d e c i m a l   p a r t  
                                                 t h i r d p a r t   =   C I n t ( s t r )  
                                                 t h i r d p a r t d i v i s o r   =   1 0  
                                                 i f   t h i r d p a r t   > =   1 0   t h e n  
                                                         t h i r d p a r t d i v i s o r   =   1 0 0  
                                                 e n d   i f  
                                                 v e r s i o n N u m b e r   =   C D b l ( v e r s i o n N u m b e r )   +   ( C D b l ( t h i r d p a r t )   /   C D b l ( t h i r d p a r t d i v i s o r ) )  
                                                 '   t h e   l i n e   b e l o w   d o e s   n o t   w o r k   o n   s y s t e m s   w h e r e   c o m m a   i s   t h e   d e c i m a l   p o i n t   ( r e p l a c e d   w i t h   l i n e   a b o v e )  
                                                 '   v e r s i o n N u m b e r   =   C d b l ( v e r s i o n N u m b e r   &   " . "   &   s t r )  
                                         c a s e   e l s e  
                                                 e x i t   f o r  
                                 e n d   s e l e c t  
                         e n d   i f  
                 n e x t  
                  
                 i f   I n S t r (   v e r s i o n S t r , " t r u n k " )   >   0     t h e n   v e r s i o n N u m b e r   =   v e r s i o n N u m b e r   +   5 0 0  
                          
  
                 g e t V e r s i o n   =   v e r s i o n N u m b e r  
  
         e n d   i f  
 e n d   f u n c t i o n  
  
  
 f u n c t i o n   T r a n s l a t e ( S t r )  
         '   h a n d l e   e r r o r   i f   T r a n s l a t e S t r   i s   n o t   d e f i n e d  
         o n   e r r o r   r e s u m e   n e x t  
         T r a n s l a t e   =   A p p l i c a t i o n . T r a n s l a t e S t r ( S t r )  
         o n   e r r o r   g o t o   0  
  
         i f   I s E m p t y ( T r a n s l a t e )   t h e n   
                 T r a n s l a t e   =   S t r  
         e n d   i f   
 e n d   f u n c t i o n  
  
 '  
 ' R e t u r n s   i n p u t   n u m b e r   r o u n d e d   t o   s p e c i f i e d   n u m b e r   o f   s i g n i f i c a n t   f i g u r e s .  
 '  
 F u n c t i o n   R o u n d S F ( d b l I n p u t ,   i n t S F )  
  
         i f   I s E m p t y ( d b l I n p u t )   O R   d b l I n p u t   =   0   t h e n  
                 R o u n d S F   =   d b l I n p u t  
         e l s e  
  
                 ' - -   S t o r e   s i g n   o f   d b l I n p u t   - -  
                 i n t S i g n   =   S g n ( d b l I n p u t )  
  
                 ' - -   C a l c u l a t e   e x p o n e n t   o f   d b l I n p u t   - -  
                 i n t C o r r P o w e r   =   I n t ( L o g 1 0 ( A b s ( d b l I n p u t ) ) )  
  
                 R o u n d S F   =   R o u n d ( d b l I n p u t   *   1 0   ^   ( ( i n t S F   -   1 )   -   i n t C o r r P o w e r ) )       ' i n t e g e r   v a l u e   w i t h   n o   s i g   f i g  
                 R o u n d S F   =   R o u n d S F   *   1 0   ^   ( i n t C o r r P o w e r   -   ( i n t S F   -   1 ) )                   ' r a i s e   t o   o r i g i n a l   p o w e r  
  
                 ' - -   R e c o n s i t u t e   f i n a l   a n s w e r   - -  
                 R o u n d S F   =   R o u n d S F   *   i n t S i g n  
         e n d   i f  
 E n d   F u n c t i o n  
  
 F u n c t i o n   L o g 1 0 ( V a l u e )  
         L o g 1 0   =   L o g ( V a l u e )   /   L o g ( 1 0 )  
 E n d   F u n c t i o n  
  
 '   R e t u r n s   m i n   o f   3   n u m b e r s ,   i g n o r i n g   e m p t y   v a l u e s  
 '   I f   a l l   3   n u m b e r s   a r e   E m p t y ,   E m p t y   i s   r e t u r n e d  
 F u n c t i o n   M i n 3 ( A ,   B ,   C )  
  
         D i m   V a l u e s ( 3 )  
         V a l u e s ( 0 ) = A  
         V a l u e s ( 1 ) = B  
         V a l u e s ( 2 ) = C  
  
         S m a l l e s t   =   E m p t y  
  
         F o r   E a c h   V a l   I n   V a l u e s  
                 I f   N o t   I s E m p t y ( V a l )   t h e n                  
                         I f   I s E m p t y ( S m a l l e s t )   O R   S m a l l e s t   >   V a l   t h e n  
                                 S m a l l e s t   =   V a l  
                         E n d   I f  
                 E n d   I f  
         N e x t  
         M i n 3 = S m a l l e s t  
  
 E n d   F u n c t i o n  
  
 '  
 '   F u n c t i o n   G e t C h a r t R G B C o l o r ( )  
 '   R e t u r n   a n   R G B   c o l o r   v a l u e   d e f i n i t i o n   b a s e d   o n   t h e   i n d e x .   T h e   i n d e x   i s   u s e d   t o   r e t u r n   u n i q u e   c o l o r  
 '   v a l u e s   o n   d i f f e r e n t   c a l l s   t o   G e t C h a r t R G B C o l o r .  
 '   I n d e x   s h o u l d   b e   g r e a t e r   t h a n   0  
 '  
 F u n c t i o n   G e t C h a r t R G B C o l o r ( I n d e x )  
         '   C o l o r   D e f i n i t i o n s  
         '   B L U E ,         R E D ,             G R E E N ,         P U R P L E ,    
         '   C Y A N ,         Y E L L O W ,       M A G E N T A ,     V I O L E T  
         '   O R A N G E ,     B R O W N ,         C O R A L ,         M A R O O N ,    
         '   O L I V E ,       T E A L ,           S E A G R E E N ,   L I M E ,  
         '   L T G R E E N ,   M I D N I G H T ,   S K Y ,             B E I G E ,  
         '   G R A Y 1 0 ,     G R A Y 2 0 ,       G R A Y 3 0 ,       G R A Y 4 0 ,  
         '   G R A Y 5 0 ,     G R A Y 6 0 ,       G R A Y 7 0 ,       G R A Y 8 0 , 	            
         '   G R A Y 9 0 ,     L T R E D ,         B L A C K  
          
          
          
  
         C h a r t C o l o r s   =   A r r a y (   " R 0 , G 0 , B 2 5 5 " ,           " R 2 5 5 , G 0 , B 0 " ,           " R 0 , G 2 5 5 , B 0 " ,           " R 1 2 8 , G 0 , B 1 2 8 " ,   _  
                                         " R 0 , G 2 5 5 , B 2 5 5 " ,       " R 2 5 5 , G 2 5 5 , B 0 " ,       " R 2 5 5 , G 0 , B 2 5 5 " ,       " R 1 2 8 , G 0 , B 2 5 5 " ,   _  
                                         " R 2 5 5 , G 1 0 2 , B 0 " ,       " R 1 5 3 , G 5 1 , B 0 " ,         " R 2 5 5 , G 0 , B 1 2 8 " ,       " R 1 2 8 , G 0 , B 0 " ,   _  
                                         " R 1 2 8 , G 1 2 8 , B 0 " ,       " R 0 , G 1 2 8 , B 1 2 8 " ,       " R 0 , G 2 5 5 , B 1 2 8 " ,       " R 0 , G 2 5 5 , B 0 " ,   _  
                                         " R 1 2 8 , G 2 5 5 , B 0 " ,       " R 0 , G 0 , B 1 2 8 " ,           " R 0 , G 1 2 8 , B 2 5 5 " ,       " R 2 5 5 , G 1 2 8 , B 0 " ,   _  
                                         " R 2 3 0 , G 2 3 0 , B 2 3 0 " ,   " R 2 0 5 , G 2 0 5 , B 2 0 5 " ,   " R 1 7 9 , G 1 7 9 , B 1 7 9 " ,   " R 1 5 2 , G 1 5 2 , B 1 5 2 " ,   _  
                                         " R 1 2 8 , G 1 2 8 , B 1 2 8 " ,   " R 1 0 2 , G 1 0 2 , B 1 0 2 " ,   " R 7 6 , G 7 6 , B 7 6 " ,         " R 5 1 , G 5 1 , B 5 1 " ,   _  
                                         " R 2 5 , G 2 5 , B 2 5 " ,         " R 2 5 5 , G 9 2 , B 9 2 " ,       " R 0 , G 0 , B 0 "   )  
  
  
         i f   G e t V a r ( " N o Y e l l o w T r a c e s " )   =   1   t h e n    
                 C h a r t C o l o r s ( 5 )   =   " R 0 , G 0 , B 0 "    
         e n d   i f  
          
         i f   G e t V a r ( " N o G r e e n T r a c e s " )   =   1   t h e n    
                 C h a r t C o l o r s ( 2 )   =   " R 1 2 8 , G 1 2 8 , B 0 "    
         e n d   i f    
    
         G e t C h a r t R G B C o l o r   =   C h a r t C o l o r s (   (   i n d e x   -   1   )   M o d   3 1 )    
 E n d   F u n c t i o n  
  
 '  
 '   F u n c t i o n   G e t C h a r t L i n e T y p e ( )  
 '   R e t u r n   a n   C h a r t   l i n e   t y p e   b a s e d   o n   t h e   i n d e x .   T h e   i n d e x   i s   u s e d   t o   r e t u r n   d i f f e r e n t   l i n e   t y p e  
 '   v a l u e s   o n   d i f f e r e n t   c a l l s   t o   G e t C h a r t R G B C o l o r .  
 '   D e f a u l t   i s   a   L i n e   t y p e   s t r i n g   t h a t   w i l l   b e   r e t u r n e d   o n c e   t h e   l i n e   t y p e s   r u n   o u t .  
 '   I f   D e f a u l t   i s   E m p t y ,   t h e   l i n e   t y p e s   w i l l   c y c l e   t h r o u g h   t h e   a v a i l a b l e   l i s t .  
 '   I n d e x   s h o u l d   b e   g r e a t e r   t h a n   0  
 '  
 F u n c t i o n   G e t C h a r t L i n e T y p e ( I n d e x ,   D e f a u l t )  
         L i n e T y p e s   =   A r r a y ( " D A S H " ,   " D O T " ,   " D A S H D O T " ,   " D A S H D O T D O T " ,   " S O L I D "   )  
         D i m   S e l e c t e d L i n e  
  
         I f   (   i n d e x   >   5   )   T h e n  
 	 I f   I s E m p t y   (   D e f a u l t   )     T h e n  
 	         S e l e c t e d L i n e   =   L i n e T y p e s   (   (   i n d e x   -   1   )   M o d   5   )  
 	 E l s e  
 	         S e l e c t e d L i n e   =   D e f a u l t  
 	 E n d   I f  
         E l s e  
 	 S e l e c t e d L i n e   =   L i n e T y p e s   (   i n d e x   -   1   )  
         E n d   I f  
  
         G e t C h a r t L i n e T y p e   =   S e l e c t e d L i n e  
 E n d   F u n c t i o n  
  
 '  
 '   F u n c t i o n   G e t C h a r t S y m b o l ( )  
 '   R e t u r n   a n   C h a r t   s y m b o l   b a s e d   o n   t h e   i n d e x .   T h e   i n d e x   i s   u s e d   t o   r e t u r n   d i f f e r e n t   s y m b o l    
 '   v a l u e s   o n   d i f f e r e n t   c a l l s   t o   G e t C h a r t R G B C o l o r .  
 '   D e f a u l t   i s   a   s y m b o l   s t r i n g   t h a t   w i l l   b e   r e t u r n e d   o n c e   t h e   s y m b o l s   r u n   o u t .  
 '   I f   D e f a u l t   i s   E m p t y ,   t h e   s y m b o l s   w i l l   c y c l e   t h r o u g h   t h e   a v a i l a b l e   l i s t .  
 '   I n d e x   s h o u l d   b e   g r e a t e r   t h a n   0  
 '  
 F u n c t i o n   G e t C h a r t S y m b o l ( I n d e x ,   D e f a u l t )  
         D i m   S e l e c t e d S y m b o l  
         S y m b o l s   =   A r r a y ( " S Y M _ C I R C L E " , " S Y M _ U P T R I A N G L E " , " S Y M _ D O W N T R I A N G L E " , " S Y M _ S Q U A R E " , " S Y M _ D I A M O N D " )  
  
         I f   (   i n d e x   >   5   )   T h e n  
 	 I f   I s E m p t y   (   D e f a u l t   )     T h e n  
 	         S e l e c t e d S y m b o l   =   S y m b o l s (   (   i n d e x   -   1   )   M o d   5 )  
 	 E l s e  
 	         S e l e c t e d S y m b o l   =   D e f a u l t  
 	 E n d   I f  
         E l s e  
 	 S e l e c t e d S y m b o l   =   S y m b o l s (   i n d e x   -   1   )  
         E n d   I f  
  
         G e t C h a r t S y m b o l   =   S e l e c t e d S y m b o l  
 E n d   F u n c t i o n  
  
 '    
 '   F u n c t i o n   G e t C h a r t C o m m a n d s ( )  
 '  
 '   R e t u r n s   a   c h a r t   c o m m a n d   s t r i n g   t h a t   c a n   b e   p a s s e d   t o   a   c h a r t   v i a   ! c m d   t o   d y n a m i c a l l y   d r a w   t r a c e s .  
 '   T h e   t r a c e s   w i l l   h a v e   d i f f e r e n t   c o l o r s   a n d   l i n e   t y p e s .  
 '  
 '   P a r a m e t e r s :  
 '  
 '   T r a c e N a m e :  
 '                       U s e d   t o   s e t   t h e   T r a c e   N a m e .   I f   m o r e   t h a n   o n e   t r a c e   i s   g e n e r a t e d ,   T h e   T r a c e   N a m e   w i l l  
 '                       h a v e   t h e   t r a c e   n u m b e r   a p p e n d e d   t o   i t .   E g :   T r a c e N a m e   =   " I m p e d e n c e " ,   t h e   t r a c e   n a m e s    
 '                       w i l l   b e   " I m p e d e n c e   1 " ,   " I m p e d e n c e   2 "   e t c .  
 '  
 '   S t a r t T r a c e N u m :  
 ' 	         T h e   t r a c e   n u m b e r   t o   s t a r t   a t .   E g :   I f   t h e   c h a r t   a l r e a d y   h a s   o n e   t r a c e   o n   i t ,   t r a c e  
 '                       n u m b e r   c a n   b e   s e t   t o   2   t o   a v o i d   o v e r w r i t i n g   t h a t   t r a c e .    
 '  
 '   X R a n g e :    
 '                       T h e   x   r a n g e   d e f i n i t i o n .   E g :   C e l l _ # ! 1 - N u m C e l l s    
 '  
 '   Y R a n g e :    
 '                       T h e   y   r a n g e   d e f i n i t i o n .   E g :   I m p e d e n c e # ! 1 - N u m C e l l s  
 '  
 '   N u m T r a c e s :  
 '                       T h e   n u m b e r   o f   t r a c e s   t o   g e n e r a t e .  
 '  
 '   S c a l e G r o u p :  
 '                       T h e   s c a l e   g r o u p   f o r   t h e   t r a c e .   ( s h o u l d   b e   1   o r   2 )  
 '  
 '   I n c l u d e S y m b o l s :  
 '                       W h e t h e r   s y m b o l s   s h o u l d   b e   i n c l u d e d   i n   t h e   t r a c e  
 '  
 ' N o t e s   o n   X R a n g e   a n d   Y R a n g e :    
 '       1 )   D o n ' t   u s e   t h e   :   c h a r a c t e r .  
 '       2 )   A n y   % T R A C E _ N U M %   i n   t h e   R a n g e   d e f i n i t i o n   s t r i n g s   w i l l   b e   r e p l a c e d   b y   t h e   t r a c e   n u m b e r .  
 '             e g :   I f   Y R a n g e =   " d b _ I m p e d e n c e _ # _ % T R A C E _ N U M % ! 1 - N u m C e l l s " ,     t h e   f i r s t   t r a c e   w i l l   h a v e   a  
 '             Y R a n g e   o f   " d b _ I m p e d e n c e _ # _ 1 ! 1 - N u m C e l l s "   a n d   t h e   s e c o n d   Y R a n g e   w i l l   h a v e    
 '             " d b _ I m p e d e n c e _ # _ 2 ! 1 - N u m C e l l s "   a n d   s o   o n .  
 '  
 ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' '  
 '   T r a c e   C o m m a n d   F o r m a t :  
 '    
 '   T h e   t r a c e   c o m m a n d   f o r m a t   a c c e p t e d   b y   c h a r t ! c m d   i s :  
 '  
 '   T R C = T r a c e   N u m b e r , T r a c e   N a m e , X   R a n g e , Y   R a n g e ,   R 2 5 5 ,   G 2 5 5 ,   B 1 2 2 ,   P o i n t   T y p e ,   L i n e   T y p e ,   S c a l e   G r o u p    
 '  
 '   X   R a n g e   f o r m a t :   $ V a r N a m e _ # ! S t a r t - E n d .     S t a r t   a n d   E n d   c a n   b e   c o n s t a n t s   o r   v a r i a b l e s .  
 '                                   T h e   d o l l a r   s i g n   p r e f i x   m e a n s   t h e   t a g   v a l u e s   w i l l   b e   s t r i n g s .  
 '  
 '   Y   R a n g e   f o r m a t :   S i m i l a r   t o   t h e   X   R a n g e   f o r m a t ,   b u t   $   i s   n o t   s u p p o r t e d .  
 '  
 '   P o i n t   T y p e :   o n e   o f   S Y M _ C I R C L E , S Y M _ N O N E , S Y M _ C I R C L E , S Y M _ U P T R I A N G L E , S Y M _ D O W N T R I A N G L E , S Y M _ S Q U A R E , S Y M _ D I A M O N D  
 '   L i n e   T y p e :   o n e   o f   S O L I D ,   D A S H D O T D O T ,   D A S H D O T ,   D O T ,   D A S H  
 '   S c a l e   G r o u p :   1   o r   2  
 '  
 F u n c t i o n   G e t C h a r t T r a c e C o m m a n d s ( T r a c e N a m e ,   S t a r t T r a c e N u m ,   X R a n g e ,   Y R a n g e ,   N u m T r a c e s ,   S c a l e G r o u p ,   I n c l u d e S y m b o l s )  
         C h a r t C m d   =   " "  
  
         T r a c e N u m   =   S t a r t T r a c e N u m  
          
         F o r   i   =   1   T o   N u m T r a c e s  
  
 	 C o l o r S t r   =   G e t C h a r t R G B C o l o r ( i )  
  
                 I f   I n c l u d e S y m b o l s   T h e n  
                         S y m b o l   =   G e t C h a r t S y m b o l ( i ,   E m p t y )  
                 E l s e  
             	         S y m b o l   =   " S Y M _ N O N E "  
                 E n d   I f  
 	  
 	 '   G o   t h r o u g h   l i n e   t y p e s   a n d   d e f a u l t   t o   " S O L I D "   o n c e   t h e   t y p e s   r u n   o u t .  
 	 L i n e T y p e   =   G e t C h a r t L i n e T y p e ( i ,   " S O L I D " )  
  
         T r a c e O f f s e t   =   G e t V a r ( " T r a c e O f f s e t " )  
 	 X S e r i e s   =   R e p l a c e ( X R a n g e ,   " % T R A C E _ N U M % " ,   C S t r ( i )   +   T r a c e O f f s e t )  
 	 Y S e r i e s   =   R e p l a c e ( Y R a n g e ,   " % T R A C E _ N U M % " ,   C S t r ( i )   +   T r a c e O f f s e t )  
  
 	 C h a r t C m d   =   C h a r t C m d   &   " T R C = "   &   T r a c e N u m   &   " , "   &   T r a c e N a m e   &   i   &   "   , "   &   X S e r i e s   &   " , "   &   Y S e r i e s   &   _  
 	                                         " , "   &   C o l o r S t r   &   " , "   &   S y m b o l   &   " , "   &   L i n e T y p e   &   " , "   &   S c a l e G r o u p   &   " : "  
  
 	 T r a c e N u m   =   T r a c e N u m   +   1  
         N e x t  
  
         G e t C h a r t T r a c e C o m m a n d s = C h a r t C m d 	      
 E n d   F u n c t i o n  
  
 s u b   P o s t O n U p d a t e ( )  
         D e f L i s t   =   G e t V a r ( " A u t o D e f i c i e n c y L i s t " )  
         i f   D e f L i s t   < >   E m p t y   t h e n    
                 D e f L i s t   =   " T e s t ( s )   "   +   D e f L i s t   +   "   F a i l e d "  
         e l s e    
                 D e f L i s t   =   "   "  
         e n d   i f    
  
     '     c a l l   S e t D e f i c i e n c i e s I n t e r n a l (   " C o m m e n t s 1 "   ,   D e f L i s t )  
 e n d   s u b    
 '  
 '   C a l c u l a t e   t h e   e r r o r   f o r   t h e   f i e l d   ' E r r T a g '    
 '   i f   t h e   e r r o r   i s   l a r g e r   t h a n   T o l e r a n c e ,   c o l o r   i t   r e d   a n d   a d d   i t   t o   t h e   d e f i c i a n c y   l i s t  
 '  
 s u b   C a l c E r r o r T a g (   E r r T a g ,   A c t u a l , D e s i r e d , T o l e r a n c e , D e f N o t e )  
         F g C o l o r   =   C L R _ B L A C K  
         E r r o r V a l   =   E m p t y  
         i f   I s N u m e r i c ( A c t u a l )   a n d   I s N u m e r i c ( D e s i r e d )   t h e n  
                 i f   n o t   I s E m p t y ( A c t u a l )   a n d   n o t   I s E m p t y ( D e s i r e d )   a n d   D e s i r e d < >   0   t h e n  
                         E r r o r V a l     =   a b s (   A c t u a l   -   D e s i r e d   )   /   D e s i r e d   *   1 0 0    
                         i f   E r r o r V a l   >   T o l e r a n c e   t h e n    
                                 C a l l   A d d F a i l u r e ( D e f N o t e )  
                                 F g C o l o r   =   C L R _ R E D  
                         e n d   i f    
                 e n d   i f    
         e n d   i f    
         c a l l   S e t V a r ( E r r T a g , E r r o r V a l )  
         c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   E r r T a g ,   " F g C o l o r       "   +   C S t r ( F g C o l o r )   )  
 e n d   s u b    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   C a l c u l a t e   t h e   e r r o r   f o r   t h e   f i e l d   ' E r r T a g '   b a s e d   o n   t h e   ' A c t u a l T a g '   a n d   ' D e s i r e d T a g '  
 '  
 '   i f   t h e   e r r o r   i s   l a r g e r   t h a n   T o l e r a n c e ,   c o l o r   i t   r e d   a n d   a d d   i t   t o   t h e   d e f i c i e n c y   l i s t   a n d   u s e   t h e    
 '   ' D e f N o t e '   t o   a n o t a t e   t h e   d e f i c i e n c y  
 '    
 '  
 '   D o   t h i s   f o r   t a g s   w i t h   a   s u f i x   o f   t h e   r a n g e   ' S t a r t N d x '   t o   ' E n d N d x '  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   C a l c E r r o r R a n g e (   E r r T a g ,   A c t u a l T a g , D e s i r e d T a g , T o l e r a n c e , D e f N o t e , S t a r t N d x ,   E n d N d x )  
          
         M s g   =   " "  
         b F a i l e d   =   f a l s e    
         f o r   N d x   =   S t a r t N d x   t o   E n d N d x  
                 F g C o l o r   =   C L R _ B L A C K  
                 E r r o r V a l   =   E m p t y  
                 A c t u a l   =   G e t N d x V a r ( A c t u a l T a g , N d x )  
                 D e s i r e d   =   G e t N d x V a r ( D e s i r e d T a g , N d x )  
                  
                 M s g   =   M s g   +   C S t r ( A c t u a l )   +   "     x x x   "   +   C S t r ( D e s i r e d )  
                  
                 i f   I s N u m e r i c ( A c t u a l )   a n d   I s N u m e r i c ( D e s i r e d )   t h e n  
                         i f   n o t   I s E m p t y ( A c t u a l )   a n d   n o t   I s E m p t y ( D e s i r e d )   a n d   D e s i r e d < >   0   t h e n  
                                 E r r o r V a l     =   a b s (   A c t u a l   -   D e s i r e d   )   /   D e s i r e d   *   1 0 0    
                                 i f   E r r o r V a l   >   T o l e r a n c e   t h e n    
                                         b F a i l e d   =   t r u e    
                                         F g C o l o r   =   C L R _ R E D  
                                 e n d   i f    
                         e n d   i f    
         e n d   i f    
               N d x E r r T a g   =   E r r T a g   +   " _ "   +   C S t r ( N d x )  
                 c a l l   S e t V a r ( N d x E r r T a g   , E r r o r V a l )  
                 c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   N d x E r r T a g   ,   " F g C o l o r       "   +   C S t r ( F g C o l o r )   )  
         n e x t    
         '  
         '   I f   a n y   o n e   o f   t h e   t e s t   f a i l ,   f l a g   t h e   o v e r a l l   t e s t   a s   a   f a i l u r e    
         '  
         i f   b F a i l e d   t h e n    
                 C a l l   A d d F a i l u r e ( D e f N o t e )  
         e n d   i f    
  
 e n d   s u b    
  
                          
                         '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   C a l c u l a t e   t h e   e r r o r   f o r   t h e   f i e l d   ' V a l u e '   b a s e d   o n   t h e   m i n   a n d   m a x   v a l u e s    
 '  
 '   i f   t h e   e r r o r   i s   l a r g e r   t h a n   T o l e r a n c e ,   c o l o r   i t   r e d   a n d   a d d   i t   t o   t h e   d e f i c i e n c y   l i s t   a n d   u s e   t h e    
 '   ' D e f N o t e '   t o   a n o t a t e   t h e   d e f i c i e n c y  
 '    
 '  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n     C h e c k E r r o r (   A c t u a l T a g ,   M i n V a l , M a x V a l , D e f N o t e )  
          
          
          
         C h e c k E r r o r   =   t r u e  
          
         M a x D e p t h   =   G e t V a r ( " F o r m M a x C a l c D e p t h " )  
         i f   I s E m p t y ( M a x D e p t h )   t h e n   M a x D e p t h   =   1  
         '  
         '   O n l y   d o   f o r   t h e   l a s t   e v a l u a t i o n   d e p t h  
         '  
         i f   F o r m . F o r m C u r C a l c D e p t h   =   M a x D e p t h     t h e n  
                  
                 M s g   =   " "  
                 F g C o l o r   =   C L R _ B L A C K  
  
                 A c t u a l   =   G e t V a r ( A c t u a l T a g )  
                                  
                 i f   I s N u m e r i c ( M i n V a l )   a n d   I s N u m e r i c ( M a x V a l )   a n d   I s N u m e r i c ( A c t u a l )   t h e n  
                         i f   n o t   I s E m p t y ( M i n V a l )   a n d   n o t   I s E m p t y ( M a x V a l )   a n d   n o t   I s E m p t y (   A c t u a l   )     t h e n  
                                 i f   A c t u a l   <   M i n V a l   o r   A c t u a l   >   M a x V a l   t h e n    
                                         C h e c k E r r o r   =   f a l s e    
                                         F g C o l o r   =   C L R _ R E D  
                                          
                                         i f   n o t   I s B l a n k S t r i n g (   D e f N o t e   )   t h e n    
                                                 C a l l   A d d F a i l u r e ( D e f N o t e )    
                                         e n d   i f    
                                 e n d   i f    
                         e n d   i f    
                 e n d   i f    
                 c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   A c t u a l T a g   ,   " F g C o l o r       "   +   C S t r ( F g C o l o r )   )  
       e n d   i f    
 e n d   f u n c t i o n    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   C a l c u l a t e   t h e   m i n i m u m   v a l u e   u s i n g   b o t h   a   p e r c e n t a g e   a n d   a n   a b s o l u t e   o f f s e t    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
  
 s u b   C a l c M i n M a x ( M i n T a g ,   M a x T a g ,   T h e o ,   P e r c e n t V a l ,   A b s V a l   )  
         C a l c M i n   =   e m p t y  
         C a l c M a x   =   e m p t y    
         i f   I s N u m e r i c (   T h e o )   a n d   I s N u m e r i c ( P e r c e n t V a l )   a n d   I s N u m e r i c ( A b s V a l )   t h e n  
                 i f   n o t   I s E m p t y ( T h e o )   a n d   n o t   I s E m p t y (   P e r c e n t V a l   )   a n d   N o t   i s E m p t y (   A b s V a l )   t h e n  
                         C a l c M i n   =   T h e o   *   (   1   -   ( P e r c e n t V a l   /   1 0 0 )   )   -   A b s V a l    
                         C a l c M a x   =   T h e o   *   (   1   + ( P e r c e n t V a l   /   1 0 0 )   )   +   A b s V a l    
                 e n d   i f    
         e n d   i f    
         c a l l   S e t V a r (   M i n T a g ,   C a l c M i n   )  
         c a l l   S e t V a r ( M a x T a g , C a l c M a x )  
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   C a l c u l a t e   t h e   m i n i m u m   v a l u e   u s i n g   b o t h   a   p e r c e n t a g e   a n d   a n   a b s o l u t e   o f f s e t   w i t h   p l u s   a n d   m i n u s  
 '   S e t   t h e   d e f   n o t e   a n d   c h a n g e   t h e   c o l o r   t o   r e d   i f   o u t   o f   t o l e r a n c e  
 '  
 '   R e t u r n s   t r u e   i f   i t   p a s s e d  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n   C a l c M i n M a x A n d E r r o r P m ( R e s u l t T a g ,   M i n T a g ,   M a x T a g ,   T h e o   ,   P e r c e n t P l u s , P e r c e n t M i n u s ,   A b s P l u s , A b s M i n u s ,   D e f N o t e   )  
  
         C a l c M i n   =   E m p t y  
         C a l c M a x   =   E m p t y  
          
         i f   I s E m p t y (   A b s P l u s   )   t h e n   A b s P l u s   =   0  
         i f   I s E m p t y (   P e r c e n t M i n u s   )   t h e n   P e r c e n t M i n u s   =   P e r c e n t P l u s  
         i f   I s E m p t y (   A b s M i n u s   )   t h e n   A b s M i n u s   =   A b s P l u s  
  
         i f   I s N u m e r i c (   T h e o )   a n d   I s N u m e r i c ( P e r c e n t M i n u s )   a n d   I s N u m e r i c ( A b s P l u s )   t h e n  
                 i f   n o t   I s E m p t y ( T h e o )   a n d   n o t   I s E m p t y (   P e r c e n t P l u s   )     t h e n  
                         C a l c M i n   =   T h e o   *   (   1   -   ( P e r c e n t M i n u s   /   1 0 0 )   )   -   A b s M i n u s    
                         C a l c M a x   =   T h e o   *   (   1   + ( P e r c e n t P l u s   /   1 0 0 )   )   +   A b s P l u s  
                 e n d   i f    
         e n d   i f    
         c a l l   S e t V a r (   M i n T a g ,   C a l c M i n   )  
         c a l l   S e t V a r ( M a x T a g , C a l c M a x )  
          
         C a l c M i n M a x A n d E r r o r P m   =   C h e c k E r r o r ( R e s u l t T a g ,   C a l c M i n ,   C a l c M a x ,   D e f N o t e )  
          
 e n d   f u n c t i o n  
  
  
  
 s u b   C a l c M i n M a x A n d E r r o r ( M i n T a g ,   M a x T a g ,   T h e o ,   P e r c e n t V a l ,   A b s V a l ,   A f T a g ,   A l T a g ,   D e f N o t e   )  
          
         c a l l   C a l c M i n M a x ( M i n T a g ,   M a x T a g ,   T h e o ,   P e r c e n t V a l ,   A b s V a l   )  
         c a l l   C h e c k E r r o r ( A f T a g ,   G e t V a r ( M i n T a g ) ,   G e t V a r ( M a x T a g ) ,   " " )  
         c a l l   C h e c k E r r o r ( A l T a g ,   G e t V a r ( M i n T a g ) ,   G e t V a r ( M a x T a g ) ,   D e f N o t e )          
          
 e n d   s u b  
  
  
  
 s u b   C a l c E r r o r A b s R a n g e (   E r r T a g ,   A c t u a l T a g , D e s i r e d T a g , T o l e r a n c e ,   A b s T o l e r a n c e ,   D e f N o t e , S t a r t N d x ,   E n d N d x )  
          
         M s g   =   " "  
         b F a i l e d   =   f a l s e    
         f o r   N d x   =   S t a r t N d x   t o   E n d N d x  
                 F g C o l o r   =   C L R _ B L A C K  
                 E r r o r V a l   =   E m p t y  
                 A c t u a l   =   G e t N d x V a r ( A c t u a l T a g , N d x )  
                 D e s i r e d   =   G e t N d x V a r ( D e s i r e d T a g , N d x )  
                  
                   c a l l   C a l c M i n M a x ( " T m p M i n " ,   " T m p M a x " ,   D e s i r e d ,   T o l e r a n c e ,   A b s T o l e r a n c e   )  
                    
                   T m p M i n   =   G e t V a r ( " T m p M i n " )  
                   T m p M a x   =   G e t V a r ( " T m p M a x " )  
                  
                 M s g   =   M s g   +   C S t r ( A c t u a l )   +   "     x x x   "   +   C S t r ( D e s i r e d )  
                  
                 i f   I s N u m e r i c ( A c t u a l )   a n d   I s N u m e r i c ( D e s i r e d )   t h e n  
                         i f   n o t   I s E m p t y ( A c t u a l )   a n d   n o t   I s E m p t y ( D e s i r e d )   a n d   D e s i r e d < >   0   t h e n  
                                 E r r o r V a l     =   a b s (   A c t u a l   -   D e s i r e d   )   /   D e s i r e d   *   1 0 0    
                                 i f   A c t u a l   >   T m p M a x   o r   A c t u a l   <   T m p M i n   t h e n    
                                         b F a i l e d   =   t r u e    
                                         F g C o l o r   =   C L R _ R E D  
                                 e n d   i f    
                         e n d   i f    
         e n d   i f    
               N d x E r r T a g   =   E r r T a g   +   " _ "   +   C S t r ( N d x )  
                 c a l l   S e t V a r ( N d x E r r T a g   , E r r o r V a l )  
                 c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   N d x E r r T a g   ,   " F g C o l o r       "   +   C S t r ( F g C o l o r )   )  
         n e x t    
         '  
         '   I f   a n y   o n e   o f   t h e   t e s t   f a i l ,   f l a g   t h e   o v e r a l l   t e s t   a s   a   f a i l u r e    
         '  
         i f   b F a i l e d   t h e n    
                 C a l l   A d d F a i l u r e ( D e f N o t e )  
         e n d   i f    
  
 e n d   s u b    
  
  
 f u n c t i o n   M a k e S t r F o r R u n T a g C m d ( T a g N a m e )  
  
         R e t V a l   =   G e t V a r ( T a g N a m e )  
         R e t V a l   =   R e p l a c e ( R e t V a l ,   " ' " ,   " ' ' " )  
          
         M a k e S t r F o r R u n T a g C m d   =   " ' "   +   R e t V a l   +   " ' "  
          
 e n d   f u n c t i o n  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 '   F U N C T I O N   G e t N d x V a r M i d E x ( S t r A , N d x , S t r B )  
 '  
 '     T h i s   f u n c t i o n   r e t u r n s   t h e   v a l u e   o f   a   v a r i a b l e   w i t h   a n d   i n d e x   i n   t h e   m i d d l e  
 '  
 '     I f   t h e   _ 1 _   d o e s   n o t   e x i s t ,   c h e c k   _   ( f o r   s i n g l e   r e c o r d   d o b l e   v a l u e s )    
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   G e t N d x V a r M i d E x ( S t r A I n , N d x , S t r B )  
         S t r a   =   S t r A I n  
         G e t N d x V a r M i d E x   =   " "  
         N a m e   =   S t r A   +   C S t r ( N d x )   +   S t r B  
         G e t N d x V a r M i d E x     =   G e t V a r ( N a m e )  
         i f   G e t N d x V a r M i d E x   =   E m p t y   a n d   N d x   =   1   t h e n    
                 L e n A   =   L e n ( S t r A )  
                 i f   L e n A   >   2   t h e n    
                         S t r a   =   L e f t ( S t r a , L e n A - 1 )  
                         N a m e   =   S t r A   +   S t r B  
 c a l l   a l o g ( " D o u b l e   c h e c k   "   +   N a m e )                          
                         G e t N d x V a r M i d E x     =   G e t V a r ( N a m e )  
                 e n d   i f    
         e n d   i f    
 e n d   f u n c t i o n  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   A d d F a i l u r e ( )  
 '   A d d   a   f a i l u r e   n o t e   t o   t h e   a u t o   d e f i c i e n c i e s   s e c t i o n  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 S U B   A d d F a i l u r e ( D e f N o t e )            
         D e f L i s t   =   G e t V a r ( " A u t o D e f i c i e n c y L i s t " )                                
         M a x D e p t h   =   G e t V a r ( " F o r m M a x C a l c D e p t h " )  
         i f   I s E m p t y ( M a x D e p t h )   t h e n   M a x D e p t h   =   1  
         i f   D e f L i s t   < >   E m p t y   t h e n   D e f L i s t   =   D e f L i s t   +   " , "            
         D e f L i s t   =   D e f L i s t   +   D e f N o t e                                                        
         i f   F o r m . F o r m C u r C a l c D e p t h   =   M a x D e p t h - 1     t h e n                  
                 C a l l   S e t V a r ( " A u t o D e f i c i e n c y L i s t " , D e f L i s t )                        
         e n d   i f    
 E N D   S U B                              
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t H i s t o r y D a t a ( )  
 '   G e t   t h e   h i s t o r i c a l   d a t a   f o r   t h e   t a g s   i n t o   t h e   v a r i a b l e s  
 '   d b _ < v a r i a b l e N a m e > _ # .     I t   w i l l   a l s o   u p d a t e   t h e    
 '   d b _ < v a r i a b l e N a m e > _ 0   r e c o r d   w i t h   t h e   t a g   v a l u e    
 '   t o   p l o t   t h e   p r e s e n t   r e s u l t s   w i t h   t h e   h i s t o r i c a l   v a l u e s  
 '  
 '   i t   a l s o   u p d a t e s . .    
 '         d b _ < V a r i a b l e N a m e > _ A v g  
 '         d b _ < V a r i a b l e N a m e > _ M i n  
 '         d b _ < V a r i a b l e N a m e > _ M a x  
 '         d b _ < V a r i a b l e N a m e > _ S t d  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   G e t H i s t o r y D a t a (   T a g s   )  
          
         i f   F o r m . F o r m L o a d e d V a l u e s   a n d   G e t V a r ( " _ _ H i s t o r y L o a d e d " )   < >   1   t h e n  
                 c a l l   s e t V a r ( " _ _ H i s t o r y L o a d e d " , 1 )  
                 '    
                 '   N u l l   a n y   o l d   r e c o r d s  
                 '    
                 D i m   p a r t s ( 1 0 0 )  
                 c o u n t   =   P a r s e S t r i n g ( T a g s ,   p a r t s ,   " , " ,   1 0 0 )  
                  
                 f o r   i   =   1   t o   C o u n t  
                         f o r   H i s t o r y   =   1   t o   1 0    
                                 c a l l   S e t N d x V a r ( " d b _ "   +   p a r t s ( i ) , H i s t o r y , E m p t y )  
                         n e x t    
                 n e x t  
                  
                 c a l l   a p p l i c a t i o n . G e t H i s t D a t a F o r T a g s ( f o r m . f o r m i d , T a g s )  
                  
         e n d   i f    
  
         '  
         '   U p d a t e   a n y   n e w l y   e n t e r e d   t a g s   t o   t h e   d b _ < T a g N a m e > _ 0    
         '   v a r i a b l e    
         c a l l   U p d a t e D b Z e r o ( T a g s )  
  
  
  
 e n d   s u b    
                                                                          
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e D b Z e r o ( )  
 '   U p d a t e s   t h e   d b _ < v a r i a b l e N a m e > _ 0   r e c o r d s   w i t h    
 '   t h e   t a g   v a l u e s   t o   p l o t   t h e   p r e s e n t   r e s u l t s   w i t h   t h e    
 '   h i s t o r i c a l   v a l u e s  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   U p d a t e D b Z e r o (   T a g s   )  
                 D i m   p a r t s ( 1 0 0 )  
                 c o u n t   =   P a r s e S t r i n g ( T a g s ,   p a r t s ,   " , " ,   1 0 0 )  
                  
                 c a l l   a l o g ( " C o u n t   =   "   +   C S t r ( C o u n t )   )  
                 f o r   i   =   1   t o   C o u n t  
                         T a g   =   P a r t s ( i )  
                         V a l u e   =   G e t V a r ( T a g )  
                         c a l l   S e t N d x V a r ( " d b _ "   +   T a g , 0 , V a l u e )  
                 n e x t  
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   H H M M t o S e c s ( )  
 '   C o n v e r t   a   H H : M M   s t r i n g   t o   s e c o n d s .  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
  
 f u n c t i o n   H H M M t o S e c s ( T i m e S t r )  
         d i m   p a r t s ( 2 )  
         V a l u e   =   0  
         c o u n t   =   P a r s e S t r i n g ( T i m e S t r ,   p a r t s ,   " : " ,   2 )          
         f o r   i   =   1   t o   c o u n t  
                 V a l u e   =   V a l u e   +     T o N u m b e r (   p a r t s ( i )   )  
                 V a l u e   =   V a l u e   *   6 0  
         n e x t  
         h h M M t o S e c s   =   V a l u e  
 e n d   f u n c t i o n  
  
        
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F U N C T I O N   S e c s T o H H M M S S (   T i m e I n S e c o n d s   )  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
          
         T i m e I n S e c o n d s   =     C L n g (   T o N u m b e r (   T i m e I n S e c o n d s   )   )  
         H H   =   I n t (   D i v i d e (   T i m e I n S e c o n d s ,   3 6 0 0   )   )  
         M M   =   T i m e I n S e c o n d s   M o d   3 6 0 0  
         S S   =   M M   M o d   6 0  
         M M   =   I n t (   D i v i d e (   M M ,   6 0   )   )  
          
         S e c s T o H H M M S S   =   r i g h t ( " 0 " &   H H , 2 )   &   " : "   &   r i g h t ( " 0 " &   M M , 2 )   &   " : "   &   r i g h t ( " 0 " &   S S , 2 )  
          
 E N D   F U N C T I O N  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F U N C T I O N   F o r m a t T i m e H H M M S S (   T i m e S t r   )  
 '       C o n v e r t s   a   t i m e   s t r i n g   i n t o   H H : M M : S S   f o r m a t  
 '       I f   t h e   i n p u t   s t r i n g   c o n t a i n s   c o l o n ( s ) ,   i t   i s   t r e a t e d   a s   H H : M M ( : S S )   f o r m a t  
 '       O t h e r w i s e ,   t h e   i n p u t   i s   a s s u m e d   t o   b e   a   v a l u e   i n   s e c o n d s  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
         i f   (   I n S t r (   T i m e S t r ,   " : "   )   )   t h e n  
                  
                 H H S t r   =   L e f t (   T i m e S t r ,   I n S t r (   T i m e S t r ,   " : "   )   -   1   )  
                 T i m e S t r   =   M i d (   T i m e S t r ,   I n S t r (   T i m e S t r ,   " : "   )   +   1   )  
                  
                 i f (   I n S t r (   T i m e S t r ,   " : "   )   )   t h e n  
                         M M S t r   =   L e f t (   T i m e S t r ,   I n S t r (   T i m e S t r ,   " : " )   -   1   )  
                         S S S t r   =   M i d (   T i m e S t r ,   I n S t r (   T i m e S t r ,   " : "   )   +   1   )  
                 e l s e  
                         M M S t r   =   T i m e S t r  
                         S S S t r   =   " 0 0 "  
                 e n d   i f  
  
                 H H   =   I n t (   T o N u m b e r (   H H S t r   )   )  
                 M M   =   I n t (   T o N u m b e r (   M M S t r   )   )  
                 S S   =   I n t (   T o N u m b e r (   S S S t r   )   )  
                                  
                 i f (   S S   >   5 9   )   t h e n  
                         M M   =   M M   +   C I n t (   D i v i d e (   S S ,   6 0   )   )  
                         S S   =   S S   M o d   6 0  
                 e n d   i f  
                 i f (   S S   <   0   )   t h e n  
                         S S   =   0  
                 e n d   i f  
  
                 i f (   M M   >   5 9   )   t h e n  
                         H H   =   H H   +   C I n t (   D i v i d e (   M M ,   6 0   )   )  
                         M M   =   M M   M o d   6 0  
                 e n d   i f  
                 i f (   M M   <   0   )   t h e n  
                         M M   =   0  
                 e n d   i f  
          
                 i f (   H H   >   9 9   )   t h e n  
                         H H   =   9 9  
                         M M   =   5 9  
                         S S   =   5 9  
                 e n d   i f  
                 i f (   H H   <   0   )   t h e n  
                         H H   =   0  
                 e n d   i f      
              
                 H H S t r   =   r i g h t ( " 0 " &   H H , 2 )  
                 M M S t r   =   r i g h t ( " 0 " &   M M , 2 )  
                 S S S t r   =   r i g h t ( " 0 " &   S S , 2 )  
                          
                 F o r m a t T i m e H H M M S S   =   H H S t r   &   " : "   &   M M S t r   &   " : "   &   S S S t r  
  
         e l s e  
                  
                 F o r m a t T i m e H H M M S S   =   S e c s T o H H M M S S (   T i m e S t r   )  
                  
         e n d   i f  
  
 E N D   F U N C T I O N  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 '   S U B   A d d D e f i c i e n c y ( )  
 '  
 '   T h i s   m e t h o d   a d d s   a   d e f i c i e n c y   n o t e   t o   t h e   a u t o   d e f i c i e n c y   l i n e .  
 '   T h e   n o t e   i s   a d d e d   u n c h a n g e d ,   a n d   n o   " F a i l e d "   t e x t   i s   a p p e n d e d   a s   i s   d o n e  
 '   b y   A d d F a i l u r e ( ) .  
 '  
 '   T e m p o r a r i l y   a d d e d   t o   t h i s   t e m p l a t e ,   t o   s u p p o r t   o l d e r   v e r s i o n s   t h a t   d o - n o t   h a v e   t h i s  
 '   c h a n g e   i n   S t a n d a r d S c r i p t s . h  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S U B   A d d D e f i c i e n c y ( D e f N o t e )  
         D e f L i s t   =   G e t V a r ( " D i r e c t D e f i c i e n c y L i s t " )  
         i f   D e f L i s t   < >   E m p t y   t h e n   D e f L i s t   =   D e f L i s t   &   " , \ n "  
                  
         D e f L i s t   =   D e f L i s t   &   D e f N o t e  
    
         M a x D e p t h   =   G e t V a r ( " F o r m M a x C a l c D e p t h " )  
         i f   I s E m p t y ( M a x D e p t h )   t h e n   M a x D e p t h   =   1        
         i f   F o r m . F o r m C u r C a l c D e p t h   =   M a x D e p t h - 1   t h e n  
                   C a l l   S e t V a r (   " D i r e c t D e f i c i e n c y L i s t " , D e f L i s t )  
         E n d   i f  
 E N D   S U B  
  
 F U N C T I O N   H a v e A u t o D e f s ( )  
         H a v e   =   f a l s e          
  
                 P d b A u t o D e f s   =   G e t V a r ( " P d b A u t o D e f s " )  
                 A u t o D e f i c i e n c y L i s t   =   G e t V a r ( " A u t o D e f i c i e n c y L i s t " )  
                 D i r e c t D e f i c i e n c y L i s t   =   G e t V a r ( " D i r e c t D e f i c i e n c y L i s t " )  
               '   C a l l   a l o g ( " I n   H a v e A u t o   1 :   "   &   A u t o D e f i c i e n c y L i s t )  
               '   C a l l   a l o g ( " I n   H a v e A u t o   2 :   "   &   P d b A u t o D e f s )  
               '   C a l l   a l o g ( " I n   H a v e A u t o   3 :   "   &   D i r e c t D e f i c i e n c y L i s t )  
                 I f   N o t   I s B l a n k S t r i n g ( P d b A u t o D e f s )   O R   N o t   I s B l a n k S t r i n g ( A u t o D e f i c i e n c y L i s t )   O R   N o t   I s B l a n k S t r i n g ( D i r e c t D e f i c i e n c y L i s t )   T h e n  
                         H a v e   =   t r u e  
                 E n d   I f  
                  
         H a v e A u t o D e f s   =   H a v e  
            
 E N D   F U N C T I O N  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 '   S U B   P o s t O n U p d a t e 2 ( )  
 '  
 '   O v e r r i d e s   t h e   d e f a u l t   P o s t O n U p d a t e 2 ( )   b y   a d d i n g   s u p p o r t   f o r   " D i r e c t D e f i c i e n c y L i s t "  
 '  
 '   T e m p o r a r i l y   a d d e d   t o   t h i s   t e m p l a t e ,   t o   s u p p o r t   o l d e r   v e r s i o n s   t h a t   d o - n o t   h a v e   t h i s  
 '   c h a n g e   i n   S t a n d a r d S c r i p t s . h  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S U B   P o s t O n U p d a t e 2 N o t A n y m o r e ( )  
       D i s p I d   =   F o r m . G e t D i s p I d ( " A u t o D e f i c i e n c y _ 1 " )  
       i f   D i s p I d   >   0   t h e n  
           D e f L i s t   =   G e t V a r ( " A u t o D e f i c i e n c y L i s t " )  
           D e f L i s t 2   =   G e t V a r ( " D e f L i s t " )  
            
           i f   D e f L i s t   < >   E m p t y   a n d   D e f L i s t 2   < >   E m p t y   t h e n   D e f L i s t   =   D e f L i s t   &   " , \ n "  
           D e f L i s t   =     D e f L i s t   &   D e f L i s t 2  
  
           I f   D e f L i s t   < >   E m p t y     T h e n  
                   D e f L i s t   =   D e f L i s t   +   T L a n g ( "   F a i l e d " )  
           E n d   I f  
            
           D e f L i s t 3   =   G e t V a r ( " D i r e c t D e f i c i e n c y L i s t " )            
           i f   D e f L i s t   < >   E m p t y     a n d   D e f L i s t 3   < >   E m p t y   t h e n   D e f L i s t   =   D e f L i s t   &   " , \ n "  
           D e f L i s t   =     D e f L i s t   &   D e f L i s t 3  
            
           D e f L i s t 4   =   G e t V a r ( " P d b A u t o D e f s " )            
           i f   D e f L i s t   < >   E m p t y     a n d   D e f L i s t 4   < >   E m p t y   t h e n   D e f L i s t   =   D e f L i s t   &   " , \ n "  
           D e f L i s t   =     D e f L i s t   &   D e f L i s t 4  
            
           P d b V e r s i o n   =   G e t V e r s i o n ( )    
            
           I f   D e f L i s t   < >   E m p t y   T h e n  
                   I f   P d b V e r s i o n   > =   8 0 0 0 . 2 7   t h e n                            
                           C a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   " A u t o D e f i c i e n c y _ 1 " ,   " W r a p T e x t T o T a b l e   ' "   &   D e f L i s t   &   " ' "   )  
                   E l s e  
                           C a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   " A u t o D e f i c i e n c y _ 1 " ,   " S e t W r a p p e d T e x t   ' "   &   D e f L i s t   &   " ' "   )                    
                   E n d   I f                
                      
         E n d   I f              
    
              
         i f   D e f L i s t   =     E m p t y   t h e n   c a l l   S e t V a r (   " A u t o D e f i c i e n c y _ 1 " ,   E m p t y   )  
              
 e l s e  
               C a l l   P o s t O n U p d a t e ( )  
       e n d   i f  
       ' c a l l   a l o g (   " T e s t i n g   D i s p I d   =   "   &   D i s p I d   &     "     d e f l i s t   =   "   &   D e f L i s t   )  
 E N D   S U B  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 '   S U B   P r e O n U p d a t e ( )  
 '  
 '   O v e r r i d e s   t h e   d e f a u l t   P r e O n U p d a t e ( )   b y   a d d i n g   s u p p o r t   f o r   " D i r e c t D e f i c i e n c y L i s t "  
 '  
 '   T e m p o r a r i l y   a d d e d   t o   t h i s   t e m p l a t e ,   t o   s u p p o r t   o l d e r   v e r s i o n s   t h a t   d o - n o t   h a v e   t h i s  
 '   c h a n g e   i n   S t a n d a r d S c r i p t s . h  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S U B   P r e O n U p d a t e ( )  
         c a l l   S e t V a r ( " A u t o D e f i c i e n c y L i s t " , E m p t y )  
         c a l l   S e t V a r ( " D i r e c t D e f i c i e n c y L i s t " , E m p t y )  
 E N D   S U B  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '       F U N C T I O N   G e t M i n ( )  
 '            
 '       R e t u r n s   t h e   l e s s e r   o f   t w o   n u m b e r s  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F U N C T I O N   G e t M i n (   N u m 1 ,   N u m 2   )  
          
         i f (   N u m 1   < =   N u m 2   )   t h e n  
                 G e t M i n   =   N u m 1  
         e l s e  
                 G e t M i n   =   N u m 2  
         e n d   i f  
          
 E N D   F U N C T I O N  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '       F U N C T I O N   G e t M a x ( )  
 '            
 '       R e t u r n s   t h e   g r e a t e r   o f   t w o   n u m b e r s  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F U N C T I O N   G e t M a x (   N u m 1 ,   N u m 2   )  
         i f (   N u m 1   > =   N u m 2   )   t h e n  
                 G e t M a x   =   N u m 1  
         e l s e  
                 G e t M a x   =   N u m 2  
         e n d   i f  
                    
 E N D   F U N C T I O N  
  
 F U N C T I O N   G e t H d r T e m p U n i t ( )  
  
     b F a h r e n h e i t   =   t r u e  
  
     '  
     '   C h e c k   t h e   o l d   s t r i n g   u n i t   i f   i t   d e f i n e d ,   o t h e r w i s e   u s e   t h e   n e w   i n t e g e r   u n i t .  
     '    
     h t u S t r   =   G e t V a r ( " H d r T e m p U n i t " )  
     i f   I s E m p t y ( h t u S t r )   t h e n  
         i f   ( G e t V a r ( " H d r T e m p U n i t I n t " )   =   G e t V a r ( " H d r T e m p U n i t I n t C " ) )   t h e n  
             b F a h r e n h e i t   =   f a l s e  
         e n d   i f  
     e l s e  
         i f   I n S t r ( h t u S t r ,   " C " )   t h e n    
             b F a h r e n h e i t   =   f a l s e  
         e n d   i f  
     e n d   i f  
  
     i f   b F a h r e n h e i t   t h e n  
         G e t H d r T e m p U n i t   =   " F "  
     e l s e  
         G e t H d r T e m p U n i t   =   " C "  
     e n d   i f  
  
 E N D   F U N C T I O N  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S e t T e m p e r a t u r e C  
 '  
 '   C h a n g e s   t e m p e r a t u r e   t o   t h e   g i v e n   v a l u e   ( i n   c )  
 '  
 '   H a n d l e s   c o n v e r s i o n   t o   F   i f   n e e d e d  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   S e t T e m p e r a t u r e C ( T e m p )  
  
         D i m   T e m p F ,   T e m p C ,   T e m p U n i t  
                
         T e m p F   =   F o r m a t N u m b e r (   C t o F (   T e m p   ) , 1 )  
         T e m p C   =   T e m p                  
                  
         c a l l   S e t V a r ( " T e m p e r a t u r e " ,   T e m p F )  
         c a l l   S e t V a r ( " T e m p e r a t u r e C " ,   T e m p C )  
  
         T e m p U n i t   =   G e t H d r T e m p U n i t ( )  
                  
         i f   I n S t r ( T e m p U n i t ,   " C " )   t h e n    
             c a l l   S e t V a r ( " E n t e r T e m p " ,   T e m p C )  
         e l s e  
             c a l l   S e t V a r ( " E n t e r T e m p " ,   T e m p F )  
         e n d   i f  
          
 e n d   s u b  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S e t T e m p e r a t u r e F  
 '  
 '   C h a n g e s   t e m p e r a t u r e   t o   t h e   g i v e n   v a l u e   ( i n   F )  
 '  
 '   H a n d l e s   c o n v e r s i o n   t o   C   i f   n e e d e d  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   S e t T e m p e r a t u r e F ( T e m p )  
          
         D i m   T e m p F ,   T e m p C ,   T e m p U n i t  
  
         T e m p F   =   T e m p  
         T e m p C   =   F o r m a t N u m b e r (   F t o C (   T e m p   ) , 1 )  
                  
         c a l l   S e t V a r ( " T e m p e r a t u r e " ,   T e m p F )  
         c a l l   S e t V a r ( " T e m p e r a t u r e C " ,   T e m p C )  
  
         T e m p U n i t   =   G e t H d r T e m p U n i t ( )  
          
         i f   I n S t r (   T e m p U n i t   ,   " C "   )   t h e n    
                 c a l l   S e t V a r ( " E n t e r T e m p " ,   T e m p C )  
         e l s e  
                 c a l l   S e t V a r ( " E n t e r T e m p " ,   T e m p F )  
         e n d   i f  
          
 e n d   s u b  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t T e m p e r a t u r e C  
 '  
 '   G e t   t h e   t e m p e r a t u r e   i n   C  
 '  
 '   H a n d l e s   c o n v e r s i o n   f r o m   F   i f   n e e d e d  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   G e t T e m p e r a t u r e C ( )  
  
         D i m   T e m p C ,   T e m p U n i t  
                      
         T e m p U n i t   =   G e t H d r T e m p U n i t ( )  
         E n t e r T e m p   =   G e t V a r ( " E n t e r T e m p " )  
          
         i f   I s B l a n k S t r i n g ( T e m p U n i t )   A n d   I s B l a n k S t r i n g ( E n t e r T e m p )   T h e n  
                  
                 T e m p C   =   F t o C (   G e t V a r ( " T e m p e r a t u r e " )   )                          
                  
         e l s e i f   I n S t r (   T e m p U n i t   ,   " C "   )   t h e n  
                 T e m p C   =   E n t e r T e m p  
         e l s e  
                 T e m p C   =   F t o C (   E n t e r T e m p   )  
         e n d   i f  
          
         G e t T e m p e r a t u r e C   =   T e m p C  
          
 e n d   f u n c t i o n  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t T e m p e r a t u r e F  
 '  
 '   G e t   t e m p e r a t u r e   v a l u e   i n   F  
 '  
 '   H a n d l e s   c o n v e r s i o n   f r o m   C   i f   n e e d e d  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   G e t T e m p e r a t u r e F ( )  
  
         D i m   T e m p F ,   T e m p U n i t  
                      
         T e m p U n i t   =   G e t H d r T e m p U n i t ( )  
         E n t e r T e m p   =   G e t V a r ( " E n t e r T e m p " )  
          
         i f   I s B l a n k S t r i n g ( T e m p U n i t )   A n d   I s B l a n k S t r i n g ( E n t e r T e m p )   T h e n  
                  
                 T e m p F   =   G e t V a r ( " T e m p e r a t u r e " )  
                  
         e l s e i f   I n S t r (   T e m p U n i t   ,   " C "   )   t h e n  
                 T e m p F   =   C t o F (   E n t e r T e m p   )  
         e l s e  
                 T e m p F   =   E n t e r T e m p  
         e n d   i f  
          
 e n d   f u n c t i o n  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 '   D o M o d a l F o r m W i t h R e t u r n  
 '  
 '     L o a d s   a   f o r m   i n   a   m o d a l   w i n d o w ,   t h e n   p a u s e s   s c r i p t   u n t i l   t h e   c h i l d   f o r m   c l o s e s  
 '  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 S U B   D o M o d a l F o r m W i t h R e t u r n (   F o r m N a m e   )  
  
         P a r e n t F o r m I d   =   G e t V a r (   " F o r m I d "   )  
  
         c a l l   a p p l i c a t i o n . d o M o d a l F o r m (   F o r m N a m e   )  
  
         d o  
               s l e e p ( 1 )  
         L o o p   w h i l e   ( G e t V a r ( " F o r m I d " )   =   P a r e n t F o r m I d )  
  
         d o  
               s l e e p ( 1 )              
         L o o p   w h i l e   ( G e t V a r ( " F o r m I d " )   < >   P a r e n t F o r m I d )  
  
 E N D   S U B    
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S u b   M a p T a g s ( T a g M a p )  
 '  
 '   M a p s   e x i s t i n g   t a g s   t o   n e w   t a g   n a m e s   b a s e d   o n   p a s s e d   t a g   m a p .  
 '  
 '   P a r a m e t e r s :  
 '             T a g M a p :   N a m e   o f   t a g   m a p   a v a i l a b l e   i n   d a t a   t a g .   E g :   " T e g g T a g M a p : "  
 '                             N e e d s   t o   i n c l u d e   t h e   c o l o n   ( : ) .  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S u b   M a p T a g s ( T a g M a p )  
  
         D i m   L i n e s ,   S t a r t N d x ,   E n d N d x ,   S r c T a g ,   D e s t T a g ,   P a r t s ,   S r c V a l  
          
  
         T e g T a g M a p   =   A p p l i c a t i o n . R e a d D a t a M u l t i L i n e F r o m T a g ( F o r m . F o r m I d ,   T a g M a p )  
         L i n e s   =   S p l i t ( T e g T a g M a p ,   v b L f )  
  
         S t a r t N d x   =   L B o u n d ( L i n e s )  
         E n d N d x   =   U B o u n d ( L i n e s )  
  
         F o r   N d x   =   S t a r t N d x   T o   E n d N d x  
  
                 P a r t s   =   S p l i t ( L i n e s ( N d x ) ,   " , " )  
                 I f   U B o u n d ( P a r t s )   > =   1   T h e n  
                         S r c T a g   =   T r i m ( P a r t s ( 0 ) )  
                         D e s t T a g   =   T r i m ( P a r t s ( 1 ) )  
                          
                         I f   N o t   I s B l a n k S t r i n g ( S r c T a g )   a n d   N o t   I s B l a n k S t r i n g ( D e s t T a g )   T h e n  
                                 S r c V a l   =   G e t V a r ( S r c T a g )  
                                 ' C a l l   a L o g ( " S r c V a l   =   "   &   S r c V a l   &   " ,   D e s t T a g   =   "   &   D e s t T a g   &   " ,   S r c T a g   =   "   &   S r c T a g )  
                                 C a l l   S e t V a r ( D e s t T a g ,   S r c V a l )  
                                  
                         E n d   I f                          
                          
                 E n d   I f  
         N e x t  
  
 E n d   S u b  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S u b   T o g g l e ( )  
 '       S w a p   T a g n a m e   v a l u e   b e t w e e n   1   a n d   0  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   T o g g l e ( T a g )  
         v a l   =   G e t V a r ( T a g )  
         i f   v a l   t h e n    
                 v a l   =   0  
         e l s e  
                 v a l   =   1  
         e n d   i f    
         c a l l   S e t V a r ( T a g , v a l )  
 e n d   s u b    
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S u b   S i n k ( )  
 '       D i s p l a y   a   b u t t o n   a s   s u n k e n  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   S i n k (   T a g s   ,   S t a t e   )  
          
         i f   S t a t e   t h e n    
                 C m d   =   " S i n k   2   ,   "   &   C L R _ Y E L L O W  
         e l s e  
                 C m d   =   " S i n k   1 ,   "   &   C L R _ G R A Y 2 0  
         e n d   i f    
         c a l l   R u n T a g C m d ( T a g s , C m d )  
 e n d   s u b    
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   f u n c t i o n   G e t R o w F r o m T a g ( )  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   G e t R o w F r o m T a g ( T a g N a m e )  
     G e t R o w F r o m T a g   =   0  
     L e n g t h   =   L e n ( T a g N a m e )  
     i f   L e n g t h   >   0   t h e n  
         f o r   n d x   =   1   t o   L e n g t h  
                 i f   M i d ( T a g N a m e , N d x , 1 )   =   " _ "   t h e n  
                         U n d e r S c o r e N d x   =   N d x  
                 e n d   i f  
         n e x t  
     e n d   i f  
      
     i f   U n d e r S c o r e N d x   >   0   a n d   U n d e r S c o r e N d x   <   L e n g t h   t h e n  
 	 R e s t O f S t r   =   M i d (   T a g N a m e , U n d e r S c o r e N d x + 1 )  
         R o w N d x     =   T o N u m b e r (   R e s t O f S t r   )  
         i f   R o w N d x   <   1   t h e n   R o w N d x   =   1  
         G e t R o w F r o m T a g   =   R o w N d x  
     e n d   i f  
 e n d   f u n c t i o n  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   f u n c t i o n   a c o s ( a )  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 f u n c t i o n   A c o s ( a )  
       i f   A b s ( a ) = 1   t h e n    
               A c o s   =   ( 1 - a ) * P I / 2  
       e l s e    
               a l o g ( " a   =   "   &   a   )  
               A c o s   =   A t n ( - a / s q r (   1 - a * a   )   ) + 2 * A t n ( 1 )  
         e n d   i f  
 e n d   f u n c t i o n  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   f u n c t i o n   P o l a r D e g r e e s T o C a r t e s i a n  
 '       C o n v e r t s   p o l a r   c o o r d s   t o   c a r t e s i a n  
 '       R e t u r n s   t r u e   i f   c o n v e r s i o n   i s   s u c c e s s f u l  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 f u n c t i o n   P o l a r D e g r e e s T o C a r t e s i a n (   d M a g ,   d D e g ,   b y r e f   d X ,   b y r e f   d Y   )  
          
         P o l a r D e g r e e s T o C a r t e s i a n   =   f a l s e  
          
         i f   N O T   ( I s E m p t y ( d M a g )   O R   I s E m p t y ( d D e g ) )   t h e n  
                 d X   =   d M a g   *   c o s d ( d D e g )  
                 d Y   =   d M a g   *   s i n d ( d D e g )  
                  
                 P o l a r D e g r e e s T o C a r t e s i a n   =   t r u e  
         e n d   i f  
          
 e n d   f u n c t i o n  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   f u n c t i o n   C a r t e s i a n T o P o l a r D e g r e e s  
 '       C o n v e r t s   p o l a r   c o o r d s   t o   c a r t e s i a n  
 '       R e t u r n s   t r u e   i f   c o n v e r s i o n   i s   s u c c e s s f u l  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 f u n c t i o n   C a r t e s i a n T o P o l a r D e g r e e s (   d X ,   d Y ,   b y r e f   d M a g ,   b y r e f   d D e g   )  
          
         C a r t e s i a n T o P o l a r D e g r e e s   =   f a l s e  
          
         i f   N O T   ( I s E m p t y ( d X )   O R   I s E m p t y ( d Y ) )   t h e n  
                  
                 d M a g   =   s q r (   ( d X   *   d X )   +   ( d Y   *   d Y )   )  
                 d R a d   =   0 . 0  
  
                 i f   (   d M a g   < >   0 . 0   )   t h e n  
                         d R a d   =   A c o s (   d X   /   d M a g   )  
                          
                         i f   (   d Y   <   0 . 0   )   t h e n  
                                 d R a d   =   2 . 0   *   G e t P I ( )   -   d R a d  
                         e n d   i f  
                 e n d   i f  
                  
                 d D e g   =   ( d R a d   *   1 8 0 . 0 )   /   G e t P I ( )  
                                  
                 C a r t e s i a n T o P o l a r D e g r e e s   =   t r u e  
         e n d   i f  
          
 e n d   f u n c t i o n  
  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   f u n c t i o n   C o n v e r t C a r t e s i a n T o P o l a r D e g r e e s  
 '       C o n v e r t s   t o / f r o m   p o l a r   c o o r d s   t o   c a r t e s i a n  
 '       T o P o l a r   -   i n d i c a t e d   i f   w e   a r e   g o i n g   t o / f r o m   p o l a r  
 '       R e t u r n s   t r u e   i f   c o n v e r s i o n   i s   s u c c e s s f u l  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 f u n c t i o n   C o n v e r t C a r t e s i a n T o P o l a r D e g r e e s (   M a g O r X ,   A n g O r Y ,   b y r e f   M a g O r X O u t ,   b y r e f   A n g O r Y O u t ,   T o P o l a r   )  
          
         i f   (   T o P o l a r   )   t h e n  
                 C o n v e r t C a r t e s i a n T o P o l a r D e g r e e s   =   C a r t e s i a n T o P o l a r D e g r e e s (   M a g O r X ,   A n g O r Y ,   M a g O r X O u t ,   A n g O r Y O u t   )  
         e l s e  
                 C o n v e r t C a r t e s i a n T o P o l a r D e g r e e s   =   P o l a r D e g r e e s T o C a r t e s i a n (   M a g O r X ,   A n g O r Y ,   M a g O r X O u t ,   A n g O r Y O u t   )  
         e n d   i f          
          
 e n d   f u n c t i o n  
  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   A d d s   a u t o   d e f i c i e n c y   m e s s a g e   f o r   g i v e n   t a g .    
 '  
 '   F i r s t   c h e c k s   i f   c o r r e s p o n d i n g   a u t o m a t i o n   c a l l   i s   a v a i l a b l e ,   a n d    
 '   i f   n o t   u s e s   o l d e r   A d d D e f i c i e n c y   f u n c t i o n  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S u b   A d d A u t o D e f i c i e n c y ( T a g ,   M e s s a g e )  
          
         '   U s i n g   e r r o r   h a n d l i n g   i n s t e a d   o f   u s i n g   a   v e r s i o n   c h e c k  
         '   f o r   s p e e d ,   s i n c e   G e t V e r s i o n ( )   r e s u l t s   i n   a n   a u t o m a t i o n   c a l l    
         '  
         O n   E r r o r   R e s u m e   N e x t  
         C a l l   A p p l i c a t i o n . A d d A u t o D e f i c i e n c y ( F o r m . F o r m I d ,   T a g ,   M e s s a g e )  
         I f   E r r . N u m b e r   >   0   T h e n  
                 C a l l   A d d D e f i c i e n c y ( M e s s a g e )  
                 E r r . C l e a r  
         E n d   I f  
          
         O n   E r r o r   G o t o   0  
 E n d   S u b  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   C l e a r s   a u t o   d e f i c i e n c y   m e s s a g e   f o r   g i v e n   t a g .    
 '  
 '   O n l y   r u n   i f   c o r r e s p o n d i n g   a u t o m a t i o n   c a l l   i s   a v a i l a b l e  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S u b   C l e a r A u t o D e f i c i e n c y ( T a g )  
         O n   E r r o r   R e s u m e   N e x t  
         C a l l   A p p l i c a t i o n . C l e a r A u t o D e f i c i e n c y ( F o r m . F o r m I d ,   T a g )  
         O n   E r r o r   G o t o   0          
 E n d   S u b  
  
 '  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 '   I n i t E n u m e r a t i o n ( )  
  
 '   C r e a t e s   v a r i a b l e s   f o r   a   s e q u e n t i a l   e n u m e r a t i o n    
  
 '   E n u m L i s t   =   " R M _ , a , b , c , d "  
  
 '   c r e a t e s   t h e   f o l l o w i n g  
  
 '   R M _ A   =   0 ,     R M _ B   =   1 ,   R M _ C   =   2   ,   R M _ D   =   3  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 '  
  
 s u b   I n i t E n u m e r a t i o n (   E n u m L i s t   )  
  
         d i m   L i s t ( 1 0 0 )  
  
         S i z e   =   P a r s e S t r i n g (   E n u m L i s t , L i s t , " , " , 1 0 0 )  
  
         P r e f i x   =   L i s t ( 1 )  
  
         f o r   i   =   2   t o   S i z e  
  
                 c a l l   S e t V a r ( P r e f i x   &   L i s t ( i )   ,   i   -   2   )  
  
 '                 c a l l   a l o g ( " I n i t E n u m   "   &   P r e f i x   &   L i s t ( i )   )  
  
         n e x t  
  
 e n d   s u b  
  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t D e g r e e S y m ( )  
 '  
 '   g e t   a   d e g r e e   s y m b o l   f r o m   t h e   t r a n s l a t i o n   t a b l e   o r    
 '   b y   a s c i i   c o d e   t o   a v o i d   m u l t i - b y t e   c h a r a c t e r   p r o b l e m s  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F U N C T I O N   G e t D e g r e e S y m ( )  
     d e g   =   G e t V a r ( " T r a n s D e g r e e S y m " )  
     i f   I s E m p t y ( d e g )   t h e n  
             d e g   =   a p p l i c a t i o n . T r a n s l a t e S t r ( " d e g r e e s y m " )  
             i f   ( S t r C o m p ( d e g ,   " d e g r e e s y m " )   =   0 )   t h e n  
                     d e g   =   " "  
             e n d   i f  
             i f   (   I s B l a n k S t r i n g ( d e g )   )   t h e n  
                 d e g   =   C h r ( 1 7 6 )  
             e n d   i f        
             c a l l   S e t V a r ( " T r a n s D e g r e e S y m " ,   d e g )  
     e n d   i f  
     G e t D e g r e e S y m   =   d e g  
 E N D   F U N C T I O N  
  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S h o w P o p u p C a l e n d a r ( )  
 '  
 '   S h o w   p o p u p   c a l e n d a r .   U s e s   t h e   " R u n T a g C m d   S h o w C a l e n d a r "    
 '   i f   a v a i l a b l e ,   o r   u s e s   t h e   " C a l e n d a r _ P o p u p "   s u b f o r m  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S u b   S h o w P o p u p C a l e n d a r ( D a t e T e x t F i e l d ,   L o c a t i o n C o n t r o l )  
  
          
         i f   G e t V e r s i o n ( )   > =   1 0 0 0 0   t h e n  
                  
                 c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   L o c a t i o n C o n t r o l ,   " S h o w C a l e n d a r   "   &   D a t e T e x t F i e l d )  
         e l s e  
                 '  
                 '   G e t   t h e   c u r r e n t   v a l u e   o f   t h e   t e x t   f i e l d ,   a n d   s e t   t h a t   v a l u e   a s   t h e   i n i t i a l   v a l u e   f o r    
                 '   t h e   c a l e n d a r   f r o m   t h e   t e x t   i n p u t   f i e l d  
                 '  
                 '  
                 C a l l   S e t V a r (   " c a l p o p u p _ p i c k d a t e " ,   G e t V a r ( D a t e T e x t F i e l d )   )  
  
                 '  
                 '   S p e c i f y   t h e   n a m e   o f   t h e   c o n t r o l   w h i c h   s h o u l d   b e   a s s i g n e d   t h e   s e l e c t e d   d a t e  
                 '  
                 C a l l   S e t V a r (   " c a l p o p u p _ r e t u r n v a r " ,   D a t e T e x t F i e l d   )  
  
                 '  
                 '   D i s p l a y   t h e   p o p u p   f o r m  
                 '  
                 A p p l i c a t i o n . D o M o d a l F o r m (   " C a l e n d a r _ P o p u p "   )  
          
         e n d   i f  
          
  
 E n d   S u b  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t U T C T i m e ( )  
 '  
 '   S h o w   p o p u p   c a l e n d a r .   U s e s   t h e   " R u n T a g C m d   S h o w C a l e n d a r "    
 '   i f   a v a i l a b l e ,   o r   u s e s   t h e   " C a l e n d a r _ P o p u p "   s u b f o r m  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   G e t U T C T i m e ( )  
         L o c a l T i m e   =   n o w ( )  
  
         A c t i v e T i m e B i a s R e g K e y   =   " H K E Y _ L O C A L _ M A C H I N E \ S y s t e m \ C u r r e n t C o n t r o l S e t \ C o n t r o l \ T i m e Z o n e I n f o r m a t i o n \ A c t i v e T i m e B i a s "  
          
         O f f s e t   =   0  
          
         S e t   O S h e l l   =   C r e a t e O b j e c t ( " W S c r i p t . S h e l l " )  
         O n   E r r o r   R e s u m e   N e x t  
         O f f s e t   =   O S h e l l . R e g R e a d ( A c t i v e T i m e B i a s R e g K e y )  
         O n   E r r o r   G o t o   0  
          
         G e t U T C T I m e   =   D a t e A d d ( " n " ,   O f f s e t ,   L o c a l T i m e )  
          
 E n d   F u n c t i o n  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   F o r m a t T i m e F o r S Q L ( )  
 '  
 '   F o r m a t   t i m e   t o   u s e   i n   a n   S Q L   q u e r y .   C h e c k s   t h e   d a t a b a s e   t y p e    
 '   a n d   r e t u r n s   f o r m a t t e d   s t r i n g .  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   F o r m a t T i m e F o r S Q L ( T i m e V a l )  
          
         I f   G e t V a r ( " e A c t i v e D b T y p e " )   =   0   o r   G e t V a r ( " e A c t i v e D b T y p e " )   =   4   T h e n  
                 '  
                 ' S Q L   S e r v e r  
                 '  
                 D a t e E s c   =   " ' "  
         E l s e  
                 '    
                 ' A c c e s s  
                 '  
                 D a t e E s c   =   " # "  
         E n d   I f  
  
         F o r m a t T i m e F o r S Q L   =   D a t e E s c   &   T i m e V a l   &   D a t e E s c  
          
 E n d   F u n c t i o n  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S t a n d a r d i z e U T C T i m e ( )    
 '  
 '   T a k e s   a   U T C T i m e   a n d   r e m o v e s   a n y   r e g i o n   s p e c i f i c   f o r m a t t i n g   f r o m   i t .    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   S t a n d a r d i z e U T C T i m e ( U t c T i m e )  
          
         h r   =   H o u r ( U t c T i m e )  
         m i n   =   M i n u t e ( U t c T i m e )  
         s e c   =   S e c o n d ( U t c T i m e )  
          
         A M P M S t r   =   " A M "  
         i f   h r   > =   1 2   t h e n  
                 A M P M S t r   =   " P M "  
         e n d   i f  
  
         i f   h r   >   1 2   t h e n  
                 h r   =   h r   -   1 2  
         e n d   i f  
  
 	 S t a n d a r d i z e U T C T i m e   =   F o r m a t D a t e T i m e ( U t c T i m e ,   v b S h o r t D a t e )     &   "   "   &   h r   &   " : "   &   m i n   &   " : "   &   s e c   &   "   "   &   A M P M S t r  
          
 E n d   F u n c t i o n  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t U T C T i m e F o r S Q L ( )    
 '  
 '   G e t   U T C   t i m e   f o r m a t t e d   f o r   u s e   i n   a n   S Q L   q u e r y .    
 '   C h e c k s   t h e   d a t a b a s e   t y p e   a n d   c o r r e c t l y   f o r m a t s   t h e   s t r i n g .  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   G e t U T C T i m e F o r S Q L ( )  
          
         U t c T i m e   =   G e t U T C T i m e ( )  
         U t c S t a n d a r d i z e d T i m e   =   S t a n d a r d i z e U T C T i m e ( U t c T i m e )          
         G e t U T C T i m e F o r S Q L   =   F o r m a t T i m e F o r S Q L ( U t c S t a n d a r d i z e d T i m e )  
          
 E n d   F u n c t i o n  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   B u i l d s   a   c h a r t   l a b e l  
 '  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   B u i l d C h a r t L a b e l ( T a g N a m e ,   L a b e l S t r ,   N d x )  
  
 	 C o l o r S t r   =   G e t C h a r t R G B C o l o r T e x t ( N d x )  
         R G B C o l o r   =   G e t C h a r t R G B C o l o r I n t ( N d x )  
 	 S y m b o l S t r   =   G e t C h a r t S y m b o l T x t ( N d x ,   E m p t y )  
 	 ' L i n e T y p e   =   G e t C h a r t L i n e T y p e ( T e s t N d x ,   E m p t y )  
 	  
 	 F u l l L b l S t r   =   L a b e l S t r   &   " - "   &   C o l o r S t r   &   "   "   &   S y m b o l S t r  
          
         c a l l   S e t N d x V a r ( T a g N a m e ,   N d x ,   F u l l L b l S t r )  
  
         c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   T a g N a m e   &   " _ " &   C S t r ( N d x ) ,   " F g C o l o r       "   +   C S t r ( R G B C o l o r )   )          
      
      
 E n d   F u n c t i o n  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t C h a r t R G B C o l o r T e x t ( )    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   G e t C h a r t R G B C o l o r T e x t ( I n d e x )  
         '   C o l o r   D e f i n i t i o n s  
         C h a r t C o l o r s   =   A r r a y (       " B l u e " ,       " R e d " ,           " G r e e n " ,         " P u r p l e " ,   _  
 	 	 	 	 	 	     " C y a n " ,         " Y e l l o w " ,       " M a g e n t a " ,     " V i o l e t " , _  
 	 	 	 	 	 	     " O r a n g e " ,     " B r o w n " ,         " C o r a l " ,         " M a r o o n " ,   _  
 	 	 	 	 	 	     " O l i v e " ,       " T e a l " ,           " S e a   G r e e n " ,   " L i m e " , _  
 	 	 	 	 	 	     " L t .   G r e e n " ,   " M i d n i g h t   B l u e " ,   " S k y   B l u e " ,             " B e i g e " , _  
 	 	 	 	 	 	     " G r a y " ,     " G r a y " ,       " G r a y " ,       " G r a y " , _  
 	 	 	 	 	 	     " G r a y " ,     " G r a y " ,       " G r a y " ,       " G r a y " , 	 _            
 	 	 	 	 	 	     " G r a y " ,     " L t .   R e d " ,         " B l a c k "   )  
    
         i f   G e t V a r ( " N o Y e l l o w T r a c e s " )     =   1   t h e n    
                 C h a r t C o l o r s ( 5 )   =   " B l a c k "    
         e n d   i f  
          
         i f   G e t V a r ( " N o G r e e n T r a c e s " )     =   1   t h e n    
                 C h a r t C o l o r s ( 2 )   =   " O l i v e "    
         e n d   i f    
    
         G e t C h a r t R G B C o l o r T e x t   =   C h a r t C o l o r s (   (   i n d e x   -   1   )   M o d   3 1 )    
 E n d   F u n c t i o n  
  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t C h a r t S y m b o l T x t ( )    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   G e t C h a r t S y m b o l T x t ( I n d e x ,   D e f a u l t )  
         D i m   S e l e c t e d S y m b o l  
         S y m b o l s   =   A r r a y ( " C i r c l e " , " T r i a n g l e " , " T r i a n g l e " , " S q u a r e " , " D i a m o n d " )  
  
         I f   (   i n d e x   >   5   )   T h e n  
 	 I f   I s E m p t y   (   D e f a u l t   )     T h e n  
 	         S e l e c t e d S y m b o l   =   S y m b o l s (   (   i n d e x   -   1   )   M o d   5 )  
 	 E l s e  
 	         S e l e c t e d S y m b o l   =   D e f a u l t  
 	 E n d   I f  
         E l s e  
 	 S e l e c t e d S y m b o l   =   S y m b o l s (   i n d e x   -   1   )  
         E n d   I f  
  
         G e t C h a r t S y m b o l T x t   =   S e l e c t e d S y m b o l  
 E n d   F u n c t i o n  
  
  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t C h a r t R G B C o l o r I n t ( )    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   G e t C h a r t R G B C o l o r I n t ( I n d e x )  
         '   C o l o r   D e f i n i t i o n s  
         C h a r t C o l o r s   =   A r r a y (       C L R _ B L U E ,       C L R _ R E D ,           C L R _ G R E E N ,       C L R _ P U R P L E ,   _  
 	 	 	 	 	 	   C L R _ C Y A N ,         C L R _ Y E L L O W ,       C L R _ M A G E N T A ,     C L R _ V I O L E T , _  
 	 	 	 	 	 	     C L R _ O R A N G E ,     C L R _ B R O W N ,       C L R _ C O R A L ,         C L R _ M A R O O N ,   _  
 	 	 	 	 	 	   C L R _ O L I V E ,       C L R _ T E A L ,           C L R _ S E A G R E E N ,   C L R _ L I M E , _  
 	 	 	 	 	 	     C L R _ L T G R E E N ,   C L R _ M I D N I G H T ,   C L R _ S K Y ,           C L R _ B E I G E , _  
 	 	 	 	 	 	     C L R _ G R A Y 1 0 ,     C L R _ G R A Y 2 0 ,       C L R _ G R A Y 3 0 ,       C L R _ G R A Y 4 0 , _  
 	 	 	 	 	 	     C L R _ G R A Y 5 0 ,     C L R _ G R A Y 6 0 ,     C L R _ G R A Y 7 0 ,       C L R _ G R A Y 8 0 , 	 _            
 	 	 	 	 	 	     C L R _ G R A Y 9 0 ,     C L R _ L T R E D ,       C L R _ B L A C K   )  
          
         i f   G e t V a r ( " N o Y e l l o w T r a c e s " )   =   1   t h e n    
                 C h a r t C o l o r s ( 5 )   =   C L R _ B L A C K  
         e n d   i f  
          
         i f   G e t V a r ( " N o G r e e n T r a c e s " )     =   1   t h e n    
                 C h a r t C o l o r s ( 2 )   =   C L R _ O L I V E  
         e n d   i f    
    
         G e t C h a r t R G B C o l o r I n t   =   C h a r t C o l o r s (   (   i n d e x   -   1   )   M o d   3 1 )    
 E n d   F u n c t i o n  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e A s L e f t T a g s ( P r e f i x ,   T a g L i s t )    
 '  
 '   U p d a t e s   t h e   A s   L e f t   t a g s   b y   t a k i n g   a   l i s t   o f   A s   F o u n d   t a g s  
 '   a n d   t h e   p r e f i x   o f   t h e   A s   L e f t   s u b f o r m  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 f u n c t i o n   U p d a t e A s L e f t T a g s ( P r e f i x ,   T a g L i s t )  
          
         A F T a g s   =   S p l i t ( T a g L i s t ,   " , " )  
          
         f o r   e a c h   C u r r e n t T a g   i n   A F T a g s  
                 i f   C u r r e n t T a g   < >   e m p t y   t h e n  
                 A F T a g N a m e   =   S p l i t ( C u r r e n t T a g ,   " _ _ " )          
                         i f   A f T a g N a m e ( 1 )   < >   E m p t y   t h e n  
                                 A L T a g N a m e   =   P r e f i x   &   " _ _ "   &   A f T a g N a m e ( 1 )  
                                 c a l l   A p p l i c a t i o n . U p d a t e A s F o u n d A s L e f t ( F o r m . f o r m I d ,   C u r r e n t T a g ,   A L T a g N a m e )  
                         e n d   i f  
                 e n d   i f  
         n e x t  
          
 e n d   f u n c t i o n  
  
     mE#џўџD a t a џўџџ\
I n t e r n a t i o n a l N o t e s :  
  
 J L T   8 / 2 8 / 2 0 0 9  
 p e r   b r u c e :   S e t V i s i b l e   -   h a n d l e   m u l t i l a n g u a g e   c a s t s   o f   t r u e / f a l s e  
  
 J L T   8 / 2 1 / 2 0 0 9  
 p e r   b r u c e :   G e t R o w F r o m T a g   i n i t   t o   0   /   r e m o v e d   a l o g   f r o m   P o s t O n U p d a t e 2  
  
 D J S   8 / 1 7 / 2 0 0 9  
 A d d   A s s e t I d _ S N   t o   s a v e d   d a t a  
  
 R T A   6 / 1 7 / 2 0 0 9  
 A d d e d   S u b   M a p T a g s ( )  
  
 R T A   4 / 1 7 / 2 0 0 9  
 A d d e d   t h e   f u n c t i o n s   S e t T e m p e r a t u r e C ( ) ,   S e t T e m p e r a t u r e F ( ) ,   G e t T e m p e r a t u r e C ( ) ,   G e t T e m p e r a t u r e F ( )  
 A d d e d   D e c l a r e V a r ( " T e m p e r a t u r e C " )   t o   O n I n i t i a l U p d a t e  
  
 J L T   9 / 9 / 0 8  
 f i x e d   g e t V e r s i o n ( )   c h e c k   f o r   " t r u n k "   ( r e t v a l   i s   1   b a s e d ;   0   m e a n   n o t   f o u n d )  
  
 B A B   9 / 9 / 0 8  
 U p d a t e   L o a d C u r v e   m e t h o d   f o r   o c r  
  
 J L T   0 3 / 0 6 / 0 8  
 f i x e d   G e t V e r s i o n ( )   t o   s u p p o r t   c r e a t i o n   o f   d o u b l e   r e g a r d l e s s   o f   d e c i m a l   s e t t i n g   ( c o m m a   o r   p e r i o d )  
  
 B A B   9 / 4 / 2 0 0 8  
 A d d S a v e d V a r ( )   m e t h o d   f o r   m e t r i c s    
  
 R T A   2 / 1 6 / 2 0 0 6  
  
 A d d e d   f u n c t i o n s   g e t V e r s i o n   a n d   T r a n s l a t e .  
 g e t V e r s i o n   c a l l s   a p p l i c a t i o n . g e t V e r s i o n ,   p a r s e s   t h e   r e s u l t   a n d   s e t s   a   v a r i a b l e   n a m e d   " P d b V e r s i o n " .  
 T r a n s l a t e   c h e c k s   w h e t h e r   t h e   a p p l i c a t i o n . t r a n s l a t e S t r   m e t h o d   e x i s t s   b e f o r e   c a l l i n g   i t .   I f   i t  
 d o e s n ' t   e x i s t ,   n o   t r a n s l a t i o n   i s   d o n e .  
 : E N D  
  
 '  
 '   * * * *     D o   n o t   c h a n g e   t h e s e   i t e m s     * * * *  
 '  
 '   I f   y o u   n e e d   t h e m   t o   b e   d i f f e r e n t   p l e a s e   c o p y   a n d   p a s t e   t h e m   i n t o    
 '   t h e   U s e r D a t a   t e m p l a t e   a n d   c h a n g e   t h e m   t h e r e  
 '  
 '   e x a m p l e   o f   s c r i p t   t o   u s e   l i s t s   i n   d r o p d o w n      
 ' . t h i s ! l i s t   =   a p p l i c a t i o n . R e a d D a t a M u l t i L i n e F r o m T a g ( . F o r m I d ,   " C a b l e I n s u l a t i o n T y p e s : " )  
 '    
  
 T e s t L e a d C o n f i g :  
 U S T :   M e a s u r e   R e d ,   G r o u n d   B l u e  
 U S T :   G r o u n d   R e d ,   M e a s u r e   B l u e  
 U S T :   M e a s u r e   R e d   a n d   B l u e  
 G S T :   G r o u n d   R e d   a n d   B l u e  
 G S T :   G u a r d   R e d ,   G r o u n d   B l u e  
 G S T :   G r o u n d   R e d ,   G u a r d   B l u e  
 G S T :   G u a r d   R e d   a n d   B l u e  
 : E N D  
  
 M a n _ I n f o :  
 G E  
 A B B  
 W e s t i n g h o u s e  
 : E N D  
  
  
 C a b l e I n s u l a t i o n T y p e s :  
 T H H N  
 T W  
 T H W  
 X L P  
 X P  
 X L  
 X L P E  
 H M W - P E  
 E D  
 E P R  
 E T H E L E Y N E  
 P E  
 E P P M  
 S F - 2  
 F R E P  
 E T E E  
 P V C  
 K Y N A R  
 T E F L O N  
 T E F Z E L  
 K A P T O N  
 E P D M  
 T R X P  
 : E N D  
  
  
 B u s h T y p e C l a s L i s t :  
 O + C  
 G O   ( 2 5 - 7 6 5   k V )  
 C T F   ( 2 0 - 6 0   k V )  
 C T F   ( 8 5 - 3 3 0   k V )  
 C T K F   ( 2 0 - 6 0   k V )  
 C T K F   ( 8 5 - 3 3 0   k V )  
 B  
 F  
 L  
 L C  
 L I  
 L M  
 O F  
 O F I  
 O F M  
 S  
 S I  
 S I M  
 T   ( G E )  
 T   ( A B B )  
 U  
 C O T  
 C O T A  
 C O S  
 S O T  
 E R C   ( 1 5 - 2 3   k V )  
 P R C   ( 1 5 - 6 9   k V )  
 P O C   ( 1 5 - 7 6 5   k V )  
 P  
 P A  
 P B  
 W T x F  
 M I C . ( 2 5 - 6 9   k V )  
 M I C . ( A b o v e   6 9   k V )  
 G   ( 4 6 - 1 3 8   k V )  
 L   ( 4 6 - 1 3 8   k V )  
 L   ( 7 . 5 - 3 4 . 5   k V )  
 G K   ( 1 5 - 5 0 0   k V )  
 L K   ( 1 5 - 6 9   k V )  
 P T A O   ( 2 5 - 2 3 0   k V )  
 3 P V . F   ( 2 5 - 2 3 0   k V )  
 C O N D .   E X C E P T   O  
 O  
 O - A 1  
 O C  
 O +  
 A  
 L C R J  
 A B  
 G O B  
 G O H  
 G O A  
 G O E  
 G O M  
 G S A  
 R T K F  
 R T X F  
 G S B  
 O T H E R  
 : E N D  
  
 V o l u m e L i s t :  
 U G  
 I G  
 L  
 k g  
 L B  
 T N  
 : E N D  
  
 C o n d i t i o n L i s t :  
 A c c e p t a b l e  
 D i r t y  
 E x c e l l e n t  
 G o o d  
 N / A  
 N / T  
 N e w  
 P o o r  
 V e r y   P o o r  
 : E N D  
  
 C l e a n L i s t :  
 C  
 C / L  
 L  
 N / A  
 : E N D  
  
 '  
 '   * * * *     D E F A U L T   C O M M E N T S     * * * *  
 '  
  
 D e f a u l t C o m m e n t s :  
  
 T e s t   O K  
  
 : E N D  
  
  
 '  
 '   * * * *     D E F A U L T   D E F I C I E N C I E S     * * * *  
 '  
  
 D e f a u l t D e f i c i e n c i e s :  
  
 T e s t   F a i l e d  
  
 : E N D            џџџ џўџ0                џўџ             mE#         $2         џўџ       џџџ                           mE#џўџO n   I n i t i a l   U p d a t e џўџџv e r s i o n   =   g e t V e r s i o n  
 c a l l   S e t V a r ( " P d b V e r s i o n " ,   v e r s i o n )  
 . D e c l a r e V a r ( " P d b D e f a u l t M e t r i c " )  
 . D e c l a r e V a r ( " U s e M e t r i c " )  
 . D e c l a r e V a r ( " L a s t U s e M e t r i c " )  
 . D e c l a r e V a r ( " f o o t _ m e t e r _ s t r " )  
 . D e c l a r e V a r ( " i n c h _ c m _ s t r " )  
 . D e c l a r e V a r ( " l c _ f o o t _ m e t e r _ s t r " )  
 . D e c l a r e V a r ( " l c _ i n c h _ c m _ s t r " )  
 . D e c l a r e V a r ( " b N a r r o w F o r m a t " )  
 c a l l   S e t T a g I f E m p t y ( " D e f a u l t F r e q " , 6 0 )  
 o n   e r r o r   r e s u m e   n e x t    
 c a l l   A p p l i c a t i o n . A d d S a v e d V a r ( F o r m . F o r m I d , " U s e M e t r i c " )  
 c a l l   A p p l i c a t i o n . A d d S a v e d V a r ( F o r m . F o r m I d , " A s s e t I D _ S N " )  
 o n   e r r o r   g o t o   0  
 . D e c l a r e V a r ( " T e m p e r a t u r e C " )  
       mE#џўџ	O n   U p d a t e џўџџ'  
 '   S e t u p   m e t r i c   /   i m p e r i a l   v a r i a b l e s  
 '  
 i f   I s E m p t y (   . U s e M e t r i c )     t h e n   . U s e M e t r i c   =   . P d b D e f a u l t M e t r i c  
 i f   . U s e M e t r i c   < >   . L a s t U s e M e t r i c     o r   I s E m p t y (   . L a s t U s e M e t r i c   )   t h e n    
         . L a s t U s e M e t r i c   =   . U s e M e t r i c  
         i f   . U s e M e t r i c   t h e n    
                 t m p _ i n c h _ c m _ s t r   =   " c m "  
                 t m p _ f o o t _ m e t e r _ s t r   =   " M E T E R S "    
         e l s e    
                 t m p _ i n c h _ c m _ s t r   =   " I N C H E S "  
                 t m p _ f o o t _ m e t e r _ s t r   =   " F E E T "  
         e n d   i f    
         . f o o t _ m e t e r _ s t r   =     T r a n s l a t e (   t m p _ f o o t _ m e t e r _ s t r   )    
         . i n c h _ c m _ s t r   =     T r a n s l a t e (   t m p _ i n c h _ c m _ s t r )  
         . l c _ f o o t _ m e t e r _ s t r   =     T r a n s l a t e (   L C a s e (   t m p _ f o o t _ m e t e r _ s t r   )   )    
         . l c _ i n c h _ c m _ s t r   =     T r a n s l a t e ( L C a s e (   t m p _ i n c h _ c m _ s t r ) )  
          
 e n d   i f    
  
  
  
      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџџџџЁ$ '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e G r a p h O p t i o n s ( )  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 ' B a c k w a r d s   c o m p a t i b i l i t y ,   P r e f i x   p a r a m   w a s   n e v e r   u s e d .  
 s u b   U p d a t e G r a p h O p t i o n s ( P r e f i x ,   C h a r t ,   N u m T r a c e s )  
         c a l l   U p d a t e G r a p h O p t i o n s E x ( C h a r t ,   N u m T r a c e s )  
 e n d   s u b  
 s u b   U p d a t e G r a p h O p t i o n s E x ( C h a r t ,   N u m T r a c e s )  
         i f   G e t V a r ( C h a r t & " _ I n i t " )   =   1   t h e n    
                 i f   R u n T a g C m d ( C h a r t , " I s C h a r t " )   =   " V a l i d C h a r t "   t h e n  
                         c a l l   S e t V a r ( C h a r t & " _ I n i t " ,   0 )                    
                         i f (   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e X " )   =   0   o r   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e Y " )   =   0   )   t h e n  
                                 f o r   N d x   =   1   t o   N u m T r a c e s  
                                         G r a p h M i n X V a l   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M i n X V a l " )  
                                         G r a p h M a x X V a l   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M a x X V a l " )  
                                         G r a p h M i n Y V a l   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M i n Y V a l " )  
                                         G r a p h M a x Y V a l   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M a x Y V a l " )  
                                         i f   G e t P r e f i x V a r ( C h a r t , " b X A x i s I s T i m e " )   t h e n  
                                                 G r a p h M i n X V a l   =   G r a p h M i n X V a l   /   6 0  
                                                 G r a p h M a x X V a l   =   G r a p h M a x X V a l   /   6 0  
                                         e n d   i f  
  
                                         i f   G e t P r e f i x V a r ( C h a r t , " b Y A x i s I s T i m e " )   t h e n  
                                                 G r a p h M i n Y V a l   =   G r a p h M i n Y V a l   /   6 0  
                                                 G r a p h M a x Y V a l   =   G r a p h M a x Y V a l   /   6 0  
                                         e n d   i f  
                                  
                                         C m d S t r   =   " S e t S c a l e   "   &   N d x   &   " , "   &   C S t r ( G r a p h M i n X V a l )   &   " , "   &   C S t r ( G r a p h M i n Y V a l )   &   " , "   &   C S t r ( G r a p h M a x X V a l )   &   " , "   &   C S t r ( G r a p h M a x Y V a l )  
                                         c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )  
                                 n e x t  
                         e n d   i f  
                  
                         f o r   N d x   =   1   t o   N u m T r a c e s  
                                 i f   G e t P r e f i x V a r ( C h a r t , " b H i d e G r a p h S y m b o l s " )   =   1   t h e n  
                                         c a l l   R u n T a g C m d ( C h a r t , " S h o w T r a c e S y m b o l   " & N d x & " , 0 " )  
                                 e l s e  
                                         c a l l   R u n T a g C m d ( C h a r t , " S h o w T r a c e S y m b o l   " & N d x & " , 1 " )                                                  
                                 e n d   i f  
                         n e x t  
                          
                         c a l l   R u n T a g C m d ( C h a r t ,   " Z o o m 1 0 0 "   )  
                         C m d S t r   =   " S e t Y A u t o S c a l e   "   &   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e Y " )  
                         c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )  
  
                         C m d S t r   =   " S e t X A u t o S c a l e   "   &   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e X " )  
                         c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )                          
                 e n d   i f  
         e n d   i f  
 e n d   s u b  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e G r a p h O p t i o n s E x t r a Y A x i s ( )  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   U p d a t e G r a p h O p t i o n s E x t r a Y A x i s ( C h a r t )  
         i f   G e t V a r ( C h a r t & " _ I n i t " )   =   1   t h e n    
                  
                 i f   R u n T a g C m d ( C h a r t , " I s C h a r t " )   =   " V a l i d C h a r t "   t h e n                  
                         c a l l   S e t V a r ( C h a r t & " _ I n i t " ,   0 )  
                         i f (   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e X " )   =   0   o r   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e Y " )   =   0   )   t h e n  
                                 G r a p h M i n X V a l   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M i n X V a l " )  
                                 G r a p h M a x X V a l   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M a x X V a l " )  
                                 G r a p h M i n Y V a l L e f t   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M i n Y V a l L e f t " )  
                                 G r a p h M a x Y V a l L e f t   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M a x Y V a l L e f t " )  
                                 G r a p h M i n Y V a l R i g h t   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M i n Y V a l R i g h t " )  
                                 G r a p h M a x Y V a l R i g h t   =   G e t P r e f i x V a r ( C h a r t , " G r a p h M a x Y V a l R i g h t " )  
  
                                 i f   G e t P r e f i x V a r ( C h a r t , " b X A x i s I s T i m e " )   t h e n  
                                         G r a p h M i n X V a l   =   G r a p h M i n X V a l   /   6 0  
                                         G r a p h M a x X V a l   =   G r a p h M a x X V a l   /   6 0  
                                 e n d   i f  
  
                                 i f   G e t P r e f i x V a r ( C h a r t , " b Y A x i s I s T i m e L e f t " )   t h e n  
                                         G r a p h M i n Y V a l L e f t   =   G r a p h M i n Y V a l L e f t   /   6 0  
                                         G r a p h M a x Y V a l L e f t   =   G r a p h M a x Y V a l L e f t   /   6 0  
                                 e n d   i f  
  
                                 i f   G e t P r e f i x V a r ( C h a r t , " b Y A x i s I s T i m e R i g h t " )   t h e n  
                                         G r a p h M i n Y V a l R i g h t   =   G r a p h M i n Y V a l R i g h t   /   6 0  
                                         G r a p h M a x Y V a l R i g h t   =   G r a p h M a x Y V a l R i g h t   /   6 0  
                                 e n d   i f  
                          
                                 C m d S t r   =   " S e t S c a l e   1 , "   &   C S t r ( G r a p h M i n X V a l )   &   " , "   &   C S t r ( G r a p h M i n Y V a l L e f t )   &   " , "   &   C S t r ( G r a p h M a x X V a l )   &   " , "   &   C S t r ( G r a p h M a x Y V a l L e f t )  
                                 c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )  
  
                                 C m d S t r   =   " S e t S c a l e   2 , "   &   C S t r ( G r a p h M i n X V a l )   &   " , "   &   C S t r ( G r a p h M i n Y V a l R i g h t )   &   " , "   &   C S t r ( G r a p h M a x X V a l )   &   " , "   &   C S t r ( G r a p h M a x Y V a l R i g h t )  
                                 c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )                                  
                         e n d   i f  
                  
                         i f   G e t P r e f i x V a r ( C h a r t , " b H i d e G r a p h S y m b o l s " )   =   1   t h e n  
                                 c a l l   R u n T a g C m d ( C h a r t , " S h o w T r a c e S y m b o l   1 , 0 " )  
                         e l s e  
                                 c a l l   R u n T a g C m d ( C h a r t , " S h o w T r a c e S y m b o l   1 , 1 " )                                                  
                         e n d   i f  
                          
                         c a l l   R u n T a g C m d ( C h a r t ,   " Z o o m 1 0 0 "   )  
                         C m d S t r   =   " S e t Y A u t o S c a l e   "   &   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e Y " )  
                         c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )  
  
                         C m d S t r   =   " S e t X A u t o S c a l e   "   &   G e t P r e f i x V a r ( C h a r t ,   " b A u t o S c a l e X " )  
                         c a l l   R u n T a g C m d ( C h a r t ,   C m d S t r )                          
                 e n d   i f  
         e n d   i f  
 e n d   s u b  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S e t T a g I f E m p t y ( )  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   S e t T a g I f E m p t y ( T a g , V a l u e )  
         v a l   =   G e t v a r ( T a g )  
         i f   i s E m p t y ( V a l )   t h e n   c a l l   S e t V a r ( T a g ,   V a l u e   )  
 e n d   s u b  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e P r e f i x C u r v e ( )  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   U p d a t e P r e f i x C u r v e ( P r e f i x )  
         P r e f i x   =   T r i m U n d e r s c o r e s ( P r e f i x )  
          
  
         i f   I s C h a n g e d (   P r e f i x     &   " _ _ T y p e " )   t h e n   C h a n g e d   =   t r u e  
         i f   I s C h a n g e d (   P r e f i x     &   " _ _ M a n u f a c t " )   t h e n   C h a n g e d   =   t r u e  
         i f   I s C h a n g e d (   P r e f i x     &   " _ _ M o d e l " )   t h e n   C h a n g e d   =   t r u e  
         i f   I s C h a n g e d (   P r e f i x     &   " _ _ C u r v e " )   t h e n   C h a n g e d   =   t r u e  
         i f   I s C h a n g e d (   P r e f i x     &   " _ _ T d m " )       t h e n   C h a n g e d   =   t r u e  
         c a l l   a l o g ( " U p d a t e   P r e f i x   C u r v e   "   &   P r e f i x   &   "   C h a n g e d   =   "   &   C h a n g e d   )  
  
         i f   C h a n g e d   t h e n    
                 T y p e S t r   =   G e t V a r (   P r e f i x   &   " _ _ T y p e " )  
                 M a n u f a c t   =   G e t V a r (   P r e f i x   &   " _ _ M a n u f a c t " )  
                 M o d e l   =   G e t V a r (   P r e f i x   &   " _ _ M o d e l " )  
                 N a m e   =   G e t V a r (   P r e f i x   &   " _ _ C u r v e " )  
                 k   =   G e t V a r ( P r e f i x   &   " _ _ T d m "   )  
 c a l l   a l o g ( " c u r v e   t y p e   =   "   &   T y p e S t r   )  
  
                 c a l l     L o a d C u r v e F r o m T a b l e (   P r e f i x   ,   T y p e S t r ,   M a n u f a c t ,   M o d e l ,     N a m e   ,   k   )  
         e n d   i f    
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   T r i m U n d e r s c o r e s ( )  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n   T r i m U n d e r s c o r e s ( T x t )  
         L e n g t h   =   L e n ( T x t )  
         i f   L e n g t h   a n d   r i g h t ( T x t , 1 )   =   " _ "   t h e n   T x t   =   l e f t ( T x t , L e n g t h - 1 )  
                  
         L e n g t h   =   L e n ( T x t )  
         i f   L e n g t h   a n d   r i g h t ( T x t , 1 )   =   " _ "   t h e n   T x t   =   l e f t ( T x t , L e n g t h - 1 )  
         T r i m U n d e r s c o r e s   =   T x t  
 e n d   f u n c t i o n    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   L o a d   a   c u r v e   f r o m   t h e   c u r v e   l i b r a r y  
 '   F i l l s   P r e f i x _ _ X _ # ,   P r e f i x _ _ Y _ #   v a r i a b l e s    
 '   A l s o   s e t s   P r e f i x _ _ N u m C u r v e P o i n t s   v a r i a b l e  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   L o a d C u r v e F r o m T a b l e (   P r e f i x   ,   T y p e S t r ,   M a n u f a c t ,   M o d e l ,     N a m e   ,   k   )  
         R e v   =   G e t I n t e r f a c e R e v ( )  
         c a l l   a L o g ( " R e v   =   "   &   R e v )  
         I f   R e v   <   1 7   T h e n  
                 c a l l   L o a d C u r v e F r o m T a b l e P r e R e v 1 7 ( P r e f i x ,   T y p e S t r ,   M a n u f a c t ,   M o d e l ,     N a m e   ,   k )  
         E l s e  
                 X S c a l e M u l t   =   G e t V a r ( " x S c a l e M u l t " )  
                 I f   I s B l a n k S t r i n g ( X S c a l e M u l t )   T h e n  
                         X S c a l e M u l t   =   1  
                 E n d   I f  
                  
                 '  
                 '   S e t   t h e   e q u a t i o n   t o   b l a n k   -   S e e   b u g z i l l a   6 0 4 0   f o r   d e t a i l s  
                 '  
                 c a l l   S e t V a r ( P r e f i x   &   " _ _ E q u a t i o n " ,   " "   )  
                  
                 c a l l   A p p l i c a t i o n . L o a d C u r v e ( F o r m . F o r m I d ,   T y p e S t r ,   M a n u f a c t ,   M o d e l ,   N a m e ,   k ,   P r e f i x ,   X S c a l e M u l t ,   " " )                  
         E n d   I f  
          
 e n d   s u b  
  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   O l d   V e r s i o n   o f   L o a d C u r v e F r o m T a b l e ,   o n l y   c a l l e d   i f   A u t o m a t i n   i n t e r f a c e   r e v i s i o n   i s   <   1 7  
 '  
 '   L o a d   a   c u r v e   f r o m   t h e   c u r v e   l i b r a r y  
 '   F i l l s   P r e f i x _ _ X _ # ,   P r e f i x _ _ Y _ #   v a r i a b l e s    
 '   A l s o   s e t s   P r e f i x _ _ N u m C u r v e P o i n t s   v a r i a b l e  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   L o a d C u r v e F r o m T a b l e P r e R e v 1 7 (   P r e f i x   ,   T y p e S t r ,   M a n u f a c t ,   M o d e l ,     N a m e   ,   k   )  
          
         x S c a l e M u l t   =   1  
         L e n g t h   =   l e n ( P r e f i x )  
         i f   L e n g t h   >   2   a n d     R i g h t (   P r e f i x , 2 )   =   " _ _ "   t h e n   P r e f i x   =   L e f t ( P r e f i x , l e n g t h - 2 )  
          
         w i t h   f o r m  
 '         S e l e c t S t r   =   " S E L E C T   X D a t a , Y D a t a , b U s e E q u a t i o n , S t a r t X , E n d X , S c a l e T y p e ,   E q u a t i o n ,   E r r o r P l u s ,   E r r o r M i n u s ,   E r r o r A b s P l u s , E r r o r A b s M i n u s   f r o m   C u r v e s   w h e r e   "   _  
 '                                   &     "   C u r v e N a m e   =   ' "   &   N a m e     &     " '   A N D   C u r v e T y p e   =   ' "   &   T y p e S t r   &   " '   A N D   M a n u f a c t u r e r   =   ' "   &   M a n u f a c t   &   " ' "   &   "   A N D   M o d e l   =   ' "   &   M o d e l   &   " ' "  
         S e l e c t S t r   =   " S E L E C T   *   f r o m   C u r v e s   w h e r e   "   _  
                                   &     "   C u r v e N a m e   =   ' "   &   N a m e     &     " '   A N D   C u r v e T y p e   =   ' "   &   T y p e S t r   &   " '   A N D   M a n u f a c t u r e r   =   ' "   &   M a n u f a c t   &   " ' "   &   "   A N D   M o d e l   =   ' "   &   M o d e l   &   " ' "  
  
         c a l l   S e t V a r ( P r e f i x   &   " _ _ N u m C u r v e P o i n t s " , 0 )  
                                    
       c a l l   a l o g ( " t d m =   "   &   k   &   "       s e l e c t   =   "   &   S e l e c t S t r   )  
         c a l l   S e t V a r ( " d b _ E r r o r M s g " , E m p t y )  
         c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S e l e c t S t r , 0 )  
          
         i f   . S q l E r r o r   =   E m p t y   t h e n    
  
                 C a l l   S e t V a r (   P r e f i x   &   " _ _ E r r o r P l u s " , . d b _ E r r o r P l u s )  
                 C a l l   S e t V a r (   P r e f i x   &   " _ _ E r r o r M i n u s " , . d b _ E r r o r M i n u s )  
                 C a l l   S e t V a r (   P r e f i x   &   " _ _ E r r o r A b s P l u s " , . d b _ E r r o r A b s P l u s )  
                 C a l l   S e t V a r (   P r e f i x   &   " _ _ E r r o r A b s M i n u s " , . d b _ E r r o r A b s M i n u s )  
                 C a l l   S e t V a r (   P r e f i x   &   " _ _ E q u a t i o n " , . d b _ E q u a t i o n )  
                 C a l l   S e t V a r (   P r e f i x   &   " _ _ R e s e t E q u a t i o n " ,   G e t V a r ( " d b _ R e s e t E q u a t i o n " )   )  
                  
                 '   c a l l   a l o g ( " S e t t i n g   e q u a t i o n   ' "   &     P r e f i x   &   " _ _ E q u a t i o n '   =   "   &   . d b _ E q u a t i o n   )  
                  
                 i f   . d b _ b U s e E q u a t i o n   t h e n    
                         S t L o g L o g   =   4  
                         S t S e m i L o g X   =   2  
                         S c a l e T y p e   =   T o N u m b e r (   . d b _ S c a l e T y p e   )  
                         S t a r t X   =   . d b _ S t a r t X  
                         i f   S t a r t X   =   1   t h e n   S t a r t X   =   1 . 0 1  
                         '  
                         '   I f   l o g - l o g   o r   s e m i - l o g   X    
                         '    
                         i f   S c a l e T y p e   =   S t L o g L o g       o r   S c a l e T y p e   =   S t S e m i L o g X     t h e n    
                                 c a l l   C a l c C u r v e L o g X ( P r e f i x , S t a r t X ,   . d b _ E n d X ,   1 0 0 ,   k ,   "   y   =   "     &   . d b _ E q u a t i o n   )  
                         e l s e    
                                 c a l l   C a l c C u r v e L i n e a r ( P r e f i x , S t a r t X ,   . d b _ E n d X ,   1 0 0 ,   k ,   "   y   =   "     &   . d b _ E q u a t i o n   )  
                         e n d   i f    
                          
                 e l s e    
                         i f   . d b _ E q u a t i o n   =   " a b s "   t h e n   x S c a l e M u l t   =   G e t V a r ( " x S c a l e M u l t " )  
                  
                         f o r   i   =   1   t o   1 0 0    
                                 c a l l   S e t N d x V a r ( P r e f i x   &   " _ _ x _ "   &   N d x , i ,   E m p t y   )  
                                 c a l l   S e t N d x V a r ( P r e f i x   &   " _ _ y _ "   &   N d x , i ,   E m p t y   )  
                         n e x t  
                  
                         i f     . d b _ E r r o r M s g   < >   " "   t h e n    
                                 c a l l   a l o g ( " D B   E R R O R :   "   &   . d b _ E r r o r M s g   )  
                         e l s e i f   . d b _ N u m R e c o r d s   >   0   t h e n    
                                 c a l l   a l o g ( " x d a t a   =   "   &   . d b _ x D a t a   )  
                                 c a l l   a l o g ( " y d a t a   =   "   &   . d b _ y D a t a   )  
                                 d i m   X D a t a ( 2 0 0 )  
                                 d i m   Y D a t a ( 2 0 0 )  
                                 c a l l   P a r s e S t r i n g (   . d b _ X D a t a ,   X D a t a , " | " , 2 0 0 )  
                                 c a l l   P a r s e S t r i n g (   . d b _ Y D a t a ,   Y D a t a , " | " , 2 0 0 )  
                                  
                                                                  
                                 '  
                                 '   x S c a l e   M u l t  
                                 '    
                                 i f   x S c a l e e M u l t   =   E m p t y   t h e n   x S c a l e M u l t   =   1  
                                 '  
                                 '   C h e c k   f o r   t i m e   d i a l   d a t a  
                                 '    
                                 b T i m e D i a l D a t a   =   f a l s e    
                                 f o r   i   =   2   t o   2 0 0    
  
                                         i f   I s E m p t y (   X D a t a ( i )   )   t h e n   e x i t   f o r  
  
                                         x D a t a N   =   t o N u m b e r ( X D a t a ( i ) )  
                                         x D a t a N M i n u s 1   =   T o N u m b e r ( x d a t a ( i - 1 ) )  
  
  
                                       i f   x D a t a N   <   x D a t a N M i n u s 1   t h e n    
                                               b T i m e D i a l D a t a   =   t r u e  
                                               e x i t   f o r    
                                       e n d   i f    
                                 n e x t    
                                 c a l l   a l o g ( " T i m e D i a l D a t a   =   "   &   b T i m e D i a l D a t a   )  
                                 T d m   =   T o N u m b e r (   k   )  
                                 i f   b T i m e D i a l D a t a   a n d   T d m   < >   E m p t y   a n d   T d m   > =   . 5   a n d   T d m   <   1 2   t h e n    
                                         T d D a t a   =   a r r a y (   . 5 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 1 0 , 1 1 , 1 2 , 1 3 , 1 4 , 1 5 , 1 6 , 1 7 )  
                                         T d N d x   =   0  
                                         d i m   Y T d ( 2 0 , 1 0 0 )  
                                         P t N d x   =   1  
                                         f o r   i   =   1   t o   2 0 0    
  
                                                 x D a t a N   =   t o N u m b e r ( X D a t a ( i ) )  
                                                 x D a t a N M i n u s 1   =   T o N u m b e r ( x d a t a ( i - 1 ) )  
  
                                           '       c a l l   a l o g ( " X D a t a ( "   &   i   &   "   )   =   "   &   x D a t a N   &   " , "   &   x D a t a N M i n u s 1   )  
                                                 i f   I s E m p t y (   X D a t a ( i )   )   t h e n   e x i t   f o r  
                                                  
                                               i f   i   >   1   a n d     x D a t a N   <   x D a t a N M i n u s 1   t h e n    
                                                       T d N d x   =   T d N d x   +   1    
                                                       P t N d x   =   1  
                                             '           c a l l   a l o g ( " N e w   T i m e   D i a l "   &   x D a t a ( i )   &   " , "   &   x D a t a ( i - 1 )   ) '  
  
                                                 e n d   i f    
                                                 Y T d ( T d N d x ,   P t N d x   )   =   y D a t a ( i )  
                                                 p t N d x   =   P t N d x   +   1  
                                                 c a l l   a l o g (   " Y d a t a ( "   &   T d N d x   &   " , "   &   P t N d x   &   "   )   =   "   &   y D a t a ( i ) )  
  
                                         n e x t    
                                          
                                         f o r   i   =   0   t o   1 2    
                                               c a l l   a l o g ( " T d m   =   "   &   T d m   &   "       T d D a t a ( "   &   i   &   " )   =   "   &   T d D a t a ( i ) )  
                                               i f   T d m   < =   T d D a t a ( i )   t h e n    
                                                     N d x 1   =   i   -   1  
                                                     N d x 2   =   i    
                                                     i f   T d m   =   T d D a t a ( i )   t h e n   N d x 1   =   i  
                                                     e x i t   f o r    
                                             e n d   i f    
                                         n e x t  
                                         T d m 1   =   T d D a t a (   N d x 1 )  
                                         T d m 2   =   T d D a t a (   N d x 2 )  
                                         P e r c e n t O f 2   =   T d m   -   T d m 1    
                                         i f   P e r c e n t O f 2   t h e n   P e r c e n t O f 2   =   D i v i d e (   P e r c e n t O f 2   ,   ( T d m 2   -   T d m 1 ) )  
                                         c a l l   a l o g ( " P o f 2   =   "   &   P e r c e n t O f 2   &   "       "   &   N d x 1   &   " , "   &   N d x 2   &   "         T d m 1 2   =   "   &   T d m 1   &   " , "   &   T d m 2 )  
                                          
                                         f o r   i   =   1   t o   2 0    
                                             Y   =   Y t d (   T d m 1 , i )  
                                             i f   I s E m p t y ( Y )   t h e n    
                                                     Y D a t a ( i )   =   Y  
                                                     X D a t a ( i )   =   E m p t y    
                                                     e x i t   f o r    
                                             e n d   i f    
                                             i f   P e r c e n t O f 2   t h e n   Y   =   Y   +   ( (   Y t d ( T d m 2 , i )   -   Y   )   *   P e r c e n t O f 2   )  
                                             c a l l   a l o g (   " Y ( "   &   i   &   " )   =   "   &   Y   )  
                                               y D a t a ( i )   =   Y  
                                         n e x t    
                                 e n d   i f    
                                 b d o n e   =   f a l s e  
                                 f o r   i   =   1   t o   2 0 0    
                                         i f   I s E m p t y (   X D a t a ( i )   )   t h e n     b D o n e   =   t r u e    
                                       i f   b D o n e   t h e n    
                                                 c a l l   S e t N d x V a r ( P r e f i x   &   " _ _ x "   &   N d x , i ,   E m p t y )  
                                                 c a l l   S e t N d x V a r ( P r e f i x   &   " _ _ y "   &   N d x , i ,   E m p t y )  
                                         e l s e  
                  
                                                 c a l l   S e t N d x V a r ( P r e f i x   &   " _ _ x "   &   N d x , i ,   X D a t a ( i )   *   x S c a l e M u l t     )  
                                                 c a l l   S e t N d x V a r ( P r e f i x   &   " _ _ y "   &   N d x , i ,   y D a t a ( i )   )  
                                                 c a l l   a l o g (   i   &   " , "   &   x D a t a ( i )   &   " , "   &   y D a t a ( i )   )  
                                                 c a l l   S e t V a r ( P r e f i x   &   " _ _ N u m C u r v e P o i n t s " , i )  
                                         e n d   i f    
                                 n e x t  
                         e n d   i f    
                 e n d   i f  
         e n d   i f    
         e n d   w i t h    
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   E v a l u a t e   a   p o i n t   o n   a   c u r v e   t h a t   i s   a l r e a d y   l o a d e d    
 '  
 '   K e p t   f o r   b a c k w a r d s   c o m p a t i b i l i t y ,   c a l l s   E v a l C u r v e E x t r a P a r a m s ( )  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n   E v a l C u r v e (   C u r v e ,   M u l t ,   T d m   )  
          
         E v a l C u r v e   =   E v a l C u r v e E x t r a P a r a m s (   C u r v e ,   M u l t ,   T d m ,   " "   )  
          
 e n d   f u n c t i o n  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   E v a l u a t e   a   p o i n t   o n   a   c u r v e   t h a t   i s   a l r e a d y   l o a d e d    
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n   E v a l C u r v e E x t r a P a r a m s (   C u r v e ,   M u l t ,   T d m ,   E x t r a P a r a m s   )  
         E q u a t i o n   =   G e t V a r (   C u r v e   &   " E q u a t i o n " )  
          
         '   s e t   d e f a u l t s   f o r   p i c k u p / t a p  
         p i c k u p   =   1  
         t a p   =   1  
  
         ' o v e r r i d e   w i t h   E x t r a P a r a m s  
         i f   N o t   I s B l a n k S t r i n g ( E x t r a P a r a m s )   t h e n  
               o n   e r r o r   r e s u m e   n e x t  
               C o m m a n d L i s t   =   R e p l a c e ( E x t r a P a r a m s ,   " , " ,   " : " )  
               c a l l   E x e c u t e ( C o m m a n d L i s t )  
               o n   e r r o r   g o t o   0    
         e n d   i f  
  
         k   =   T d m  
          
            
         x   =   M u l t  
         M   =   M u l t  
         I   =   M u l t  
         y   =   E m p t y  
         o n   e r r o r   r e s u m e   n e x t  
         c a l l   E x e c u t e (   "   y   =   "   &   E q u a t i o n   )  
         o n   e r r o r   g o t o   0    
  
   c a l l   a l o g ( " E v a l C u r v e (   "   &   C u r v e   &   " , "   &   M   &   " , "   &   T d m   &   " )   =   "   &   y   &   "             E q   =   "   &   E q u a t i o n   )  
         '  
         '   I f   n o   e q u a t i o n ,   t r y   a   c h a r t   l o o k u p  
         '  
         i f   I s E m p t y ( y )   t h e n    
                 C h a r t   =   G e t V a r ( C u r v e   &   " C h a r t " )  
                 T r a c e   =   G e t V a r ( C u r v e   &   " T r a c e " )  
                 c a l l   a l o g ( " E v a l C u r v e ( " & C u r v e & " )   "   &   C h a r t   &   " , "   &   T r a c e   )  
                 i f   C h a r t   < >   E m p t y   a n d   T r a c e   < >   E m p t y   t h e n   y   =     R u n T a g C m d ( C h a r t , " L o o k u p Y   "   &   T r a c e   &   "   "   &   M u l t   )  
         e n d   i f    
  
         E v a l C u r v e E x t r a P a r a m s   =     y    
 e n d   f u n c t i o n    
  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S w e e p   t h r o u g h   a   u s e r   d e f i n e d   e q u a t i o n   w i t h   a   l o g   x   s c a l e  
 '  
 '   K e p t   f o r   b a c k w a r d s   c o m p a t i b i l i t y ,   c a l l s   C a l c C u r v e L o g X E x t r a P a r a m s ( )  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   C a l c C u r v e L o g X ( P r e f i x , S t a r t V a l , E n d V a l , N u m P o i n t s , k , E q u a t i o n )  
          
         C a l l   C a l c C u r v e L o g X E x t r a P a r a m s ( P r e f i x , S t a r t V a l , E n d V a l , N u m P o i n t s , k , E q u a t i o n , " " )  
          
 e n d   s u b  
          
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S w e e p   t h r o u g h   a   u s e r   d e f i n e d   e q u a t i o n   w i t h   a   l o g   x   s c a l e    
 '  
 '   E x t r a P a r a m s :   V a l u e s   o f   e x t r a   p a r a m e t e r s   t o   b e   m a d e   a v a i l a b l e   t o  
 '                             e q u a t i o n .   S h o u l d   b e   c o l o n   ( o r   c o m m a )   s e p a r a t e d  
 '  
 '                             e g :   " p i c k u p = 4 . 3 , x x = 1 2 " .  
 '  
 '                             N o t e :   O n l y   p i c k u p   a n d   t a p   a r e   s u p p o r t e d   f o r   n o w  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   C a l c C u r v e L o g X E x t r a P a r a m s ( P r e f i x , S t a r t V a l , E n d V a l , N u m P o i n t s , k , E q u a t i o n , E x t r a P a r a m s )  
   w i t h   F o r m  
       i f   I s N u m e r i c ( E n d V a l )   a n d   n o t   I s E m p t y ( E n d V a l )   a n d   I s N u m e r i c ( S t a r t V a l )   a n d   n o t   I s E m p t y ( S t a r t V a l )   t h e n  
  
           '   A v o i d   e r r o r s   w h e n   c a l c u l a t i n g   l o g  
           I f   E n d V a l   < =   0   T h e n  
               E n d V a l   =   1  
           E n d   I f  
           I f   S t a r t V a l   < =   0   T h e n  
               S t a r t V a l   =   0 . 0 0 1  
           E n d   I f  
  
           L o g 1 0 E n d   =   L o g 1 0 ( E n d V a l )  
           L o g 1 0 S t a r t   =   L o g 1 0 ( S t a r t V a l )  
  
           D e l t a L o g 1 0   =   L o g 1 0 E n d   -   L o g 1 0 S t a r t    
           I n c L o g 1 0   =   D e l t a L o g 1 0   /   ( N u m P o i n t s   -   1 )  
           L o g V a l   =   L o g 1 0 S t a r t  
              
           '   s e t   d e f a u l t s   f o r   p i c k u p / t a p  
           p i c k u p   =   1  
           t a p   =   1  
  
           ' o v e r r i d e   w i t h   E x t r a P a r a m s  
           I f   N o t   I s B l a n k S t r i n g ( E x t r a P a r a m s )   T h e n  
               o n   e r r o r   r e s u m e   n e x t  
               C o m m a n d L i s t   =   R e p l a c e ( E x t r a P a r a m s ,   " , " ,   " : " )  
               c a l l   E x e c u t e ( C o m m a n d L i s t )  
               o n   e r r o r   g o t o   0    
           E n d   I f  
  
           T d m   =   k  
  
           N d x   =   0  
           f o r   P o i n t   =   1   t o   N u m P o i n t s  
              
               x   =   1 0   ^   L o g V a l                
                      
               M   =   x  
               I   =   x  
               y   =   E m p t y  
               o n   e r r o r   r e s u m e   n e x t  
               c a l l   E x e c u t e (   E q u a t i o n   )  
               o n   e r r o r   g o t o   0    
  
               ' c a l l   a l o g (   " p o i n t   "   &   P o i n t   &   " f ( "   &   x   & " )   =   "   &   y   )  
               i f   N o t   I s E m p t y ( y )   t h e n    
                   N d x   =   N d x   +   1  
                              
                   c a l l   S e t N d x V a r ( P r e f i x +   " _ _ Y " , N d x , y )  
                   c a l l   S e t N d x V a r ( P r e f i x   +   " _ _ X " , N d x , x )  
                              
                   '       c a l l   a l o g (   P r e f i x   &   " _ _ X         X Y   =   "   &   i   &   " , "   &   x   &   " , "   &   y   )  
               e n d   i f    
               L o g V a l   =   L o g V a l   +   I n c L o g 1 0    
                      
           n e x t  
       e n d   i f  
       c a l l   S e t V a r ( P r e f i x   +   " _ _ N u m C u r v e P o i n t s " , N d x     )  
   e n d   w i t h    
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S w e e p   t h r o u g h   a   u s e r   d e f i n e d   e q u a t i o n   w i t h   a   l i n e a r   x   s c a l e    
 '  
 '   K e p t   f o r   b a c k w a r d s   c o m p a t i b i l i t y ,   c a l l s   C a l c C u r v e L i n e a r E x t r a P a r a m s ( )  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   C a l c C u r v e L i n e a r ( P r e f i x , S t a r t V a l , E n d V a l , N u m P o i n t s , k , E q u a t i o n )  
          
         C a l l   C a l c C u r v e L i n e a r E x t r a P a r a m s ( P r e f i x , S t a r t V a l , E n d V a l , N u m P o i n t s , k , E q u a t i o n , " " )  
 e n d   s u b  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S w e e p   t h r o u g h   a   u s e r   d e f i n e d   e q u a t i o n   w i t h   a   l i n e a r   x   s c a l e    
 '  
 '   E x t r a P a r a m s :   V a l u e s   o f   e x t r a   p a r a m e t e r s   t o   b e   m a d e   a v a i l a b l e   t o  
 '                             e q u a t i o n .   S h o u l d   b e   c o l o n   ( o r   c o m m a )   s e p a r a t e d  
 '  
 '                             e g :   " p i c k u p = 4 . 3 , x x = 1 2 " .  
 '  
 '                             N o t e :   O n l y   p i c k u p   a n d   t a p   a r e   s u p p o r t e d   f o r   n o w  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   C a l c C u r v e L i n e a r E x t r a P a r a m s ( P r e f i x , S t a r t V a l , E n d V a l , N u m P o i n t s , k , E q u a t i o n , E x t r a P a r a m s )  
     w i t h   F o r m  
         i f   I s N u m e r i c ( E n d V a l )   a n d   n o t   I s E m p t y ( E n d V a l )   a n d   I s N u m e r i c ( S t a r t V a l )   a n d   n o t   I s E m p t y ( S t a r t V a l )   t h e n  
             D e l t a     =   E n d V a l   -   S t a r t V a l    
             x   =   S t a r t V a l  
             I n c r e m e n t   =   D e l t a   /   N u m P o i n t s  
  
             '   s e t   d e f a u l t s   f o r   p i c k u p / t a p  
             p i c k u p   =   1  
             t a p   =   1  
  
             ' o v e r r i d e   w i t h   E x t r a P a r a m s  
             I f   N o t   I s B l a n k S t r i n g ( E x t r a P a r a m s )   T h e n  
                 o n   e r r o r   r e s u m e   n e x t  
                 C o m m a n d L i s t   =   R e p l a c e ( E x t r a P a r a m s ,   " , " ,   " : " )  
                 c a l l   E x e c u t e ( C o m m a n d L i s t )  
                 o n   e r r o r   g o t o   0    
             E n d   I f  
  
             f o r   P o i n t   =   1   t o   N u m P o i n t s  
  
                 c a l l   S e t N d x V a r ( P r e f i x   +   " _ _ X " , P o i n t , x )  
                  
                 M   =   x  
                 I   =   x  
                 y   =   E m p t y  
                 o n   e r r o r   r e s u m e   n e x t                  
                 c a l l   E x e c u t e (   E q u a t i o n   )  
                 o n   e r r o r   g o t o   0                    
                 c a l l   S e t N d x V a r ( P r e f i x +   " _ _ Y " , P o i n t , y )  
                 x   =   x   +   I n c r e m e n t    
                  
             n e x t  
         e n d   i f  
  
         c a l l   S e t V a r ( P r e f i x   +   " _ _ N u m C u r v e P o i n t s " , N u m P o i n t s )  
     e n d   w i t h    
 e n d   s u b    
  
  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t I n t e r f a c e R e v ( )  
 '   R e t u r n s   t h e   v e r s i o n   o f   t h e   a u t o m a t i o n   i n t e r f a c e  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   G e t I n t e r f a c e R e v ( )  
         G e t I n t e r f a c e R e v   =   0  
         o n   e r r o r   r e s u m e   n e x t    
         G e t I n t e r f a c e R e v   =     A p p l i c a t i o n . G e t I n t e r f a c e R e v ( )  
         o n   e r r o r   g o t o   0  
 e n d   f u n c t i o n  
  
 '  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   C t o F ( )  
 '   C o n v e r t   d e g r e e s   C   t o   d e g r e e s   F  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   C t o F (   D e g C )  
         C t o F   =   E m p t y  
         i f   n o t   I s E m p t y (   D e g C   )   t h e n   C t o F   =         T o N u m b e r ( D e g C )   *   9 . 0   /   5 . 0     +   3 2 . 0  
 e n d   f u n c t i o n  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   F t o C ( )  
 '   C o n v e r t   d e g r e e s   F   t o   d e g r e e s   C  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   F t o C (   D e g F )  
         F t o C   =   E m p t y  
         i f   n o t   I s E m p t y (   D e g F   )   t h e n   F t o C   =     ( T o N u m b e r ( D e g F )   -   3 2 )   *   5 . 0   /   9 . 0      
 e n d   f u n c t i o n  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   H H M M S S t o S e c s ( )  
 '   C o n v e r t   a   H H : M M : S S   s t r i n g   t o   s e c o n d s .  
 '   a l s o   s u p p o r t s   M M : S S   a n d   S S  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
  
 f u n c t i o n   H H M M S S t o S e c s ( T i m e S t r )  
         d i m   p a r t s ( 3 )  
         V a l u e   =   0  
         c o u n t   =   P a r s e S t r i n g ( T i m e S t r ,   p a r t s ,   " : " ,   3 )  
         f o r   i   =   1   t o   c o u n t  
                 V a l u e   =   V a l u e   *   6 0  
                 V a l u e   =   V a l u e   +     T o N u m b e r (   p a r t s ( i )   )  
         n e x t  
         h h M M S S t o S e c s   =   V a l u e  
 e n d   f u n c t i o n  
          
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   I s C h a n g e d ( )  
 '   R e t u r n   t r u e   i f   t h e   v a r i a b l e   h a s   c h a n g e d   s i n c e   t h e   l a s t   c a l l    
 '   U s e   U n i q u e   k e y   f o r   m u l t i p l e   i s   c h a n g e d   f o r   s a m e   v a r i a b l e  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n   I s C h a n g e d ( V a r N a m e )  
             I s C h a n g e d   =   I s C h a n g e d E x ( V a r N a m e , " 1 " )  
 e n d   f u n c t i o n    
  
 f u n c t i o n   I s C h a n g e d E x ( V a r N a m e , K e y )  
         I s C h a n g e d E x   =   f a l s e  
         C h a n g e V a r   =   " _ _ C h a n g e d _ _ "   +   V a r N a m e   +   K e y  
         L a s t V a l u e   =   G e t V a r ( C h a n g e V a r )  
         T h i s V a l u e   =   G e t V a r ( V a r N a m e )  
  
          
          
         i f   I s E m p t y ( L a s t V a l u e )   t h e n    
                 I s C h a n g e d E x   =   t r u e  
         e l s e  
                 i f   L a s t V a l u e   =   " _ _ E m p t y _ _ "   t h e n    
                         i f   n o t   I s E m p t y ( T h i s V a l u e )   t h e n   I s C h a n g e d E x   =   t r u e  
                 e l s e    
                         i f   L a s t V a l u e   < >   T h i s V a l u e   t h e n   I s C h a n g e d E x   =   t r u e    
                 e n d   i f    
         e n d   i f    
         i f   I s C h a n g e d E x   t h e n    
                  
                 ' c a l l   a l o g (   " C h a n g e d :     "   +   C h a n g e V a r   +   "       "   +   C S t r ( L a s t V a l u e )   +   "         "   +   C S t r ( T h i s V a l u e )   )          
                  
                 i f   I s E m p t y ( T h i s V a l u e )   t h e n    
                         c a l l   S e t V a r ( C h a n g e V a r , " _ _ E m p t y _ _ " )  
                 e l s e    
                         c a l l   S e t V a r (   C h a n g e V a r , T h i s V a l u e )  
                 e n d   i f    
         e n d   i f    
 e n d   f u n c t i o n    
  
  
  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t V a r T c f (   B y V a l   D e g C ,   B y V a l   T e m p V a r N a m e ,   B y V a l   T a b l e N a m e ,   B y V a l   K C o l u m n N a m e   )  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 ' 	  
 	 	 	 	 	 	 	 	     '  
 F U N C T I O N   G e t V a r T c f (   B y V a l   D e g C ,   B y V a l   T e m p V a r N a m e ,   B y V a l   T a b l e N a m e ,   B y V a l   K C o l u m n N a m e   )  
  
 '   N O T E   T h a t   t h i s   f u n c t i o n   c a n   n o t   b e   u s e d   o n   t a b l e s   w i t h   i n t e g e r   t e m p e r a t u r e s  
  
         S e l e c t S t a t e m e n t H i g h   =   " S E L E C T   T O P   1   "   +   T e m p V a r N a m e   +   "   a s   [ T e m p C ]   ,   "   +   K C o l u m n N a m e   +   "   a s   [ K F a c t o r ]   F R O M   "   +   T a b l e N a m e   +   _  
                 "   W H E R E   "   +   T e m p V a r N a m e   +   "     > =   "   +   C s t r ( D e g C )   +   _  
                 "   O R D E R   B Y   "   +   T e m p V a r N a m e    
  
         S e l e c t S t a t e m e n t L o w   =   " S E L E C T   T O P   1   "   +   T e m p V a r N a m e   +   "   a s   [ T e m p C ]     ,   "   +   K C o l u m n N a m e   +   "   a s   [ K F a c t o r ]   F R O M   "   +   T a b l e N a m e   +   _  
                 "   W H E R E   "   +   T e m p V a r N a m e   +   "     < =   "   +   C s t r ( D e g C )   +   _  
                 "   O R D E R   B Y   "   +   T e m p V a r N a m e   +   "   D E S C "  
  
  
         K F a c t o r   =   e m p t y  
  
  
         '  
         '   I f   a   v a l i d   t e m p e r a t u r e   t o   l o o k u p  
         '  
         I F   N O T   I s E m p t y ( D e g C )   A N D   I s N u m e r i c ( D e g C )   T H E N  
  
                 '  
                 '   G e t   t h e   p o i n t   a b o v e   t h e   l o o k u p   p o i n t  
                 '  
                 F o r m . D e c l a r e V a r ( " d b _ T e m p C " )  
                 F o r m . D e c l a r e V a r ( " d b _ K F a c t o r " )  
                 c a l l   A p p l i c a t i o n . Q u e r y L o o k u p T a b l e ( F o r m . F o r m I d ,   S e l e c t S t a t e m e n t H i g h ,   0 )  
                 T e m p H i g h   =   F o r m . d b _ T e m p C  
                 K H i g h         =   F o r m . d b _ K F a c t o r  
  
                  
                 '  
                 '   G e t   t h e   f a c t o r   b e l o w   t h e   l o o k u p   p o i n t  
                 '  
                 c a l l   A p p l i c a t i o n . Q u e r y L o o k u p T a b l e ( F o r m . F o r m I d ,   S e l e c t S t a t e m e n t L o w ,   0 )  
                 T e m p L o w   =   F o r m . d b _ T e m p C  
                 K L o w         =   F o r m . d b _ K F a c t o r  
                  
                  
  
                 c a l l   a l o g (   C S t r ( T e m p L o w )   +   " , "   +   C S t r ( K L o w )   +   " , "   +   C S t r ( T e m p H i g h )   +   " , "   +   C S t r ( K H i g h ) )  
  
  
                 '  
                 '   I f   t h e   l o o k u p   p o i n t   w a s   w i t h i n   t h e   t a b l e ,   i n t e r p o l a t e   t h e   r e s u l t s  
                 '  
                 i f   T e m p L o w   < >   e m p t y   a n d   T e m p H i g h   < >   e m p t y   t h e n  
                         D e l t a T   =   T e m p H i g h   -   T e m p L o w  
                         D e l t a K   =   K H i g h   -   K L o w  
                         i f   D e l t a T   >   0   t h e n  
                                 R a t i o   =   ( D e g C   -   T e m p L o w )   /   D e l t a T  
                                 K F a c t o r   =   K L o w   +   (   D e l t a K   *   R a t i o   )  
                         e l s e  
                                 K F a c t o r   =   K L o w  
                         e n d   i f  
  
 	 	 	   F o r m . D e c l a r e V a r ( " P d b T e m p C o r r F a c t o r " )  
  
                         '  
                         '   c o n v e r t   f r o m   v t _ r 4   t o   s t r i n g ,   t o   v t _ r 8  
                         '   t o   a v o i d   c o e r s i o n   r e s o l u t i o n   p r o b l e m .  
                         '  
                         S t r T c f   =   C S t r ( K F a c t o r )  
                         K F a c t o r   =   C D b l ( S t r T c f )  
                 E N D   I F  
  
         E N D   I F  
         G e t V a r T c f   =   K F a c t o r  
 E N D   F U N C T I O N  
  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   A d d S a v e d V a r ( )  
 '  
 '   W r a p p e r   a r o u n d   t h e   a p p l i c a t i o n   c a l l   t o   i g n o r e   f o r   e a r l y   r e v s  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   A d d S a v e d V a r (     V a r N a m e   )  
         o n   e r r o r   r e s u m e   n e x t    
         c a l l   A p p l i c a t i o n . A d d S a v e d V a r ( F o r m . F o r m I d , V a r N a m e )  
         o n   e r r o r   g o t o   0  
 e n d   s u b    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e A s F o u n d A s L e f t ( N a m e )  
 '  
 '   S e t s   t h e   A l < N a m e >   v a r i a b l e   t o   t h e   A f < N a m e >   v a r i a b l e   i f   i t   i s    
 '   e m p t y    
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   U p d a t e A s F o u n d A s L e f t ( N a m e )  
         A f N a m e   =   " A f "   +   N a m e    
         A l N a m e   =   " A l "   +   N a m e    
         c a l l   A p p l i c a t i o n . U p d a t e A s F o u n d A s L e f t (   F o r m . F o r m I d ,   A f N a m e , A l N a m e )  
 e n d   S u b  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e A s F o u n d A s L e f t N d x ( N a m e , N d x )  
 '  
 '   S e t s   t h e   A l < N a m e > _ < n d x >   v a r i a b l e   t o   t h e   A f < N a m e > < n d x >   v a r i a b l e   i f   i t   i s    
 '   e m p t y    
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   U p d a t e A s F o u n d A s L e f t N d x ( N a m e ,   N d x )  
         c a l l   U p d a t e A s F o u n d A s L e f t (   N a m e   +   " _ "   +   C S t r ( N d x )   )  
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S e t N u m R o w s ( )  
 '  
 '   S e t s   t h e   n u m b e r   o f   r o w s   f o r   t h e   s p e c i f i e d   t e m p l a t e   i f   i t   h a s    
 '   c h a n g e d    
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   S e t N u m R o w s (   T e m p l a t e N a m e ,   N u m R o w s   )  
                 c a l l   A p p l i c a t i o n . R u n T a g C m d   (   F o r m . F o r m I d ,   T e m p l a t e N a m e ,   " S e t N u m R o w s   "   +   C S t r ( N u m R o w s )   )  
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S e t V i s i b l e ( )  
 '   S h o w s   o r   h i d e s   a   t e m p l a t e   o r   f i e l d    
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   S e t V i s i b l e ( N a m e ,   V i s i b l e )  
   C m d   =   " v i s i b l e   0 "  
   F a l s e S t r   =   C S t r ( f a l s e )  
   i f   n o t   I s E m p t y ( V i s i b l e )   t h e n  
       V i s S t r   =   C S t r ( V i s i b l e )  
       i f   V i s S t r   < >   F a l s e S t r   a n d   V i s S t r   < >   " 0 "   a n d   V i s S t r   < >   " - 2 "   t h e n  
           C m d   =   " v i s i b l e   1 "  
       e n d   i f  
   e n d   i f  
   c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   N a m e ,   C m d )  
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t L i s t ( )  
 '   R e a d s   a   l i s t   o f   v a l u e s   f r o m   t h e   d a t a   t a b   a n d   r e t u r n s   i t  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n   G e t L i s t ( t a g )  
         G e t L i s t   =   a p p l i c a t i o n . R e a d D a t a M u l t i L i n e F r o m T a g ( F o r m . F o r m I d ,   T a g )  
 e n d   f u n c t i o n    
  
 ' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
 '   P a r s e s   t h e   v a l u e   r e t u r n e d   b y   a p p l i c a t i o n . g e t V e r s i o n  
 '   a n d   c o n s t r u c t s   t h e   v e r s i o n   n u m b e r .  
 '   e g :   I f   g e t V e r s i o n   r e t u r n s   " 4 . 1 2 . T R U N K . 2  
 '   t h e   v e r s i o n   n u m b e r   i s :   4   *   1 0 0 0   +   1 2   +   0 . 2   =   4 0 1 2 . 2  
 ' - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  
  
 f u n c t i o n   g e t V e r s i o n  
  
         o n   e r r o r   r e s u m e   n e x t  
         ' e n a b l e   e r r o r   h a n d l i n g :   I n   c a s e   a p p l i c a t i o n . g e t V e r s i o n   i s   n o t   d e f i n e d .  
         v e r s i o n S t r   =   a p p l i c a t i o n . g e t V e r s i o n  
         ' d i s a b l e   e r r o r   h a n d l i n g  
         o n   e r r o r   g o t o   0  
  
         i f   I s E m p t y ( v e r s i o n S t r )   t h e n  
                 g e t V e r s i o n   =   E m p t y  
         e l s e  
  
                 '   C h e c k   f o r   a t   m o s t   5   d o t   ( . )   s e p a r a t e d   p a r t s   i n   v e r s i o n   s t r i n g .  
                 '   N O T E :   P a r s e S t r i n g ( )   d o e s n ' t   u s e   t h e   0 ' t h   e l e m e n t   o f   t h e   a r r a y .  
                 '               T h e r e f o r e ,   w e   c r e a t e   a n   a r r a y   o f   6   e l e m e n t s  
                 D i m   p a r t s ( 6 )  
                 c o u n t   =   P a r s e S t r i n g ( v e r s i o n S t r ,   p a r t s ,   " . " ,   5 )  
  
                 '   G o   t h r o u g h   t h e   p a r s e d   v a l u e s   a n d   g e t   t h e   f i r s t   3   n u m e r i c   v a l u e s .  
                 D i m   n  
                 D i m   v e r s i o n N u m b e r  
                 n   =   0  
  
                 f o r   i   =   1   t o   c o u n t  
                         s t r   =   p a r t s ( i )  
  
                         i f   I s N u m e r i c ( s t r )   t h e n  
                                 n   =   n   +   1  
  
                                 '   I f   t h e   f i r s t   3   n u m b e r s   a r e   4 , 1 2 , 5 ,   T h e   v e r s i o n   w i l l   b e   4 0 1 2 . 5  
                                 s e l e c t   c a s e   n  
                                         c a s e   1  
                                                 v e r s i o n N u m b e r   =   1 0 0 0   *   C I n t ( s t r )  
                                         c a s e   2  
                                                 v e r s i o n N u m b e r   =   v e r s i o n N u m b e r   +   C I n t ( s t r )  
                                         c a s e   3  
                                                  
                                                 ' T h e   t h i r d   n u m b e r   i s   t h e   d e c i m a l   p a r t  
                                                 t h i r d p a r t   =   C I n t ( s t r )  
                                                 t h i r d p a r t d i v i s o r   =   1 0  
                                                 i f   t h i r d p a r t   > =   1 0   t h e n  
                                                         t h i r d p a r t d i v i s o r   =   1 0 0  
                                                 e n d   i f  
                                                 v e r s i o n N u m b e r   =   C D b l ( v e r s i o n N u m b e r )   +   ( C D b l ( t h i r d p a r t )   /   C D b l ( t h i r d p a r t d i v i s o r ) )  
                                                 '   t h e   l i n e   b e l o w   d o e s   n o t   w o r k   o n   s y s t e m s   w h e r e   c o m m a   i s   t h e   d e c i m a l   p o i n t   ( r e p l a c e d   w i t h   l i n e   a b o v e )  
                                                 '   v e r s i o n N u m b e r   =   C d b l ( v e r s i o n N u m b e r   &   " . "   &   s t r )  
                                         c a s e   e l s e  
                                                 e x i t   f o r  
                                 e n d   s e l e c t  
                         e n d   i f  
                 n e x t  
                  
                 i f   I n S t r (   v e r s i o n S t r , " t r u n k " )   >   0     t h e n   v e r s i o n N u m b e r   =   v e r s i o n N u m b e r   +   5 0 0  
                          
  
                 g e t V e r s i o n   =   v e r s i o n N u m b e r  
  
         e n d   i f  
 e n d   f u n c t i o n  
  
  
 f u n c t i o n   T r a n s l a t e ( S t r )  
         '   h a n d l e   e r r o r   i f   T r a n s l a t e S t r   i s   n o t   d e f i n e d  
         o n   e r r o r   r e s u m e   n e x t  
         T r a n s l a t e   =   A p p l i c a t i o n . T r a n s l a t e S t r ( S t r )  
         o n   e r r o r   g o t o   0  
  
         i f   I s E m p t y ( T r a n s l a t e )   t h e n   
                 T r a n s l a t e   =   S t r  
         e n d   i f   
 e n d   f u n c t i o n  
  
 '  
 ' R e t u r n s   i n p u t   n u m b e r   r o u n d e d   t o   s p e c i f i e d   n u m b e r   o f   s i g n i f i c a n t   f i g u r e s .  
 '  
 F u n c t i o n   R o u n d S F ( d b l I n p u t ,   i n t S F )  
  
         i f   I s E m p t y ( d b l I n p u t )   O R   d b l I n p u t   =   0   t h e n  
                 R o u n d S F   =   d b l I n p u t  
         e l s e  
  
                 ' - -   S t o r e   s i g n   o f   d b l I n p u t   - -  
                 i n t S i g n   =   S g n ( d b l I n p u t )  
  
                 ' - -   C a l c u l a t e   e x p o n e n t   o f   d b l I n p u t   - -  
                 i n t C o r r P o w e r   =   I n t ( L o g 1 0 ( A b s ( d b l I n p u t ) ) )  
  
                 R o u n d S F   =   R o u n d ( d b l I n p u t   *   1 0   ^   ( ( i n t S F   -   1 )   -   i n t C o r r P o w e r ) )       ' i n t e g e r   v a l u e   w i t h   n o   s i g   f i g  
                 R o u n d S F   =   R o u n d S F   *   1 0   ^   ( i n t C o r r P o w e r   -   ( i n t S F   -   1 ) )                   ' r a i s e   t o   o r i g i n a l   p o w e r  
  
                 ' - -   R e c o n s i t u t e   f i n a l   a n s w e r   - -  
                 R o u n d S F   =   R o u n d S F   *   i n t S i g n  
         e n d   i f  
 E n d   F u n c t i o n  
  
 F u n c t i o n   L o g 1 0 ( V a l u e )  
         L o g 1 0   =   L o g ( V a l u e )   /   L o g ( 1 0 )  
 E n d   F u n c t i o n  
  
 '   R e t u r n s   m i n   o f   3   n u m b e r s ,   i g n o r i n g   e m p t y   v a l u e s  
 '   I f   a l l   3   n u m b e r s   a r e   E m p t y ,   E m p t y   i s   r e t u r n e d  
 F u n c t i o n   M i n 3 ( A ,   B ,   C )  
  
         D i m   V a l u e s ( 3 )  
         V a l u e s ( 0 ) = A  
         V a l u e s ( 1 ) = B  
         V a l u e s ( 2 ) = C  
  
         S m a l l e s t   =   E m p t y  
  
         F o r   E a c h   V a l   I n   V a l u e s  
                 I f   N o t   I s E m p t y ( V a l )   t h e n                  
                         I f   I s E m p t y ( S m a l l e s t )   O R   S m a l l e s t   >   V a l   t h e n  
                                 S m a l l e s t   =   V a l  
                         E n d   I f  
                 E n d   I f  
         N e x t  
         M i n 3 = S m a l l e s t  
  
 E n d   F u n c t i o n  
  
 '  
 '   F u n c t i o n   G e t C h a r t R G B C o l o r ( )  
 '   R e t u r n   a n   R G B   c o l o r   v a l u e   d e f i n i t i o n   b a s e d   o n   t h e   i n d e x .   T h e   i n d e x   i s   u s e d   t o   r e t u r n   u n i q u e   c o l o r  
 '   v a l u e s   o n   d i f f e r e n t   c a l l s   t o   G e t C h a r t R G B C o l o r .  
 '   I n d e x   s h o u l d   b e   g r e a t e r   t h a n   0  
 '  
 F u n c t i o n   G e t C h a r t R G B C o l o r ( I n d e x )  
         '   C o l o r   D e f i n i t i o n s  
         '   B L U E ,         R E D ,             G R E E N ,         P U R P L E ,    
         '   C Y A N ,         Y E L L O W ,       M A G E N T A ,     V I O L E T  
         '   O R A N G E ,     B R O W N ,         C O R A L ,         M A R O O N ,    
         '   O L I V E ,       T E A L ,           S E A G R E E N ,   L I M E ,  
         '   L T G R E E N ,   M I D N I G H T ,   S K Y ,             B E I G E ,  
         '   G R A Y 1 0 ,     G R A Y 2 0 ,       G R A Y 3 0 ,       G R A Y 4 0 ,  
         '   G R A Y 5 0 ,     G R A Y 6 0 ,       G R A Y 7 0 ,       G R A Y 8 0 , 	            
         '   G R A Y 9 0 ,     L T R E D ,         B L A C K  
          
          
          
  
         C h a r t C o l o r s   =   A r r a y (   " R 0 , G 0 , B 2 5 5 " ,           " R 2 5 5 , G 0 , B 0 " ,           " R 0 , G 2 5 5 , B 0 " ,           " R 1 2 8 , G 0 , B 1 2 8 " ,   _  
                                         " R 0 , G 2 5 5 , B 2 5 5 " ,       " R 2 5 5 , G 2 5 5 , B 0 " ,       " R 2 5 5 , G 0 , B 2 5 5 " ,       " R 1 2 8 , G 0 , B 2 5 5 " ,   _  
                                         " R 2 5 5 , G 1 0 2 , B 0 " ,       " R 1 5 3 , G 5 1 , B 0 " ,         " R 2 5 5 , G 0 , B 1 2 8 " ,       " R 1 2 8 , G 0 , B 0 " ,   _  
                                         " R 1 2 8 , G 1 2 8 , B 0 " ,       " R 0 , G 1 2 8 , B 1 2 8 " ,       " R 0 , G 2 5 5 , B 1 2 8 " ,       " R 0 , G 2 5 5 , B 0 " ,   _  
                                         " R 1 2 8 , G 2 5 5 , B 0 " ,       " R 0 , G 0 , B 1 2 8 " ,           " R 0 , G 1 2 8 , B 2 5 5 " ,       " R 2 5 5 , G 1 2 8 , B 0 " ,   _  
                                         " R 2 3 0 , G 2 3 0 , B 2 3 0 " ,   " R 2 0 5 , G 2 0 5 , B 2 0 5 " ,   " R 1 7 9 , G 1 7 9 , B 1 7 9 " ,   " R 1 5 2 , G 1 5 2 , B 1 5 2 " ,   _  
                                         " R 1 2 8 , G 1 2 8 , B 1 2 8 " ,   " R 1 0 2 , G 1 0 2 , B 1 0 2 " ,   " R 7 6 , G 7 6 , B 7 6 " ,         " R 5 1 , G 5 1 , B 5 1 " ,   _  
                                         " R 2 5 , G 2 5 , B 2 5 " ,         " R 2 5 5 , G 9 2 , B 9 2 " ,       " R 0 , G 0 , B 0 "   )  
  
  
         i f   G e t V a r ( " N o Y e l l o w T r a c e s " )   =   1   t h e n    
                 C h a r t C o l o r s ( 5 )   =   " R 0 , G 0 , B 0 "    
         e n d   i f  
          
         i f   G e t V a r ( " N o G r e e n T r a c e s " )   =   1   t h e n    
                 C h a r t C o l o r s ( 2 )   =   " R 1 2 8 , G 1 2 8 , B 0 "    
         e n d   i f    
    
         G e t C h a r t R G B C o l o r   =   C h a r t C o l o r s (   (   i n d e x   -   1   )   M o d   3 1 )    
 E n d   F u n c t i o n  
  
 '  
 '   F u n c t i o n   G e t C h a r t L i n e T y p e ( )  
 '   R e t u r n   a n   C h a r t   l i n e   t y p e   b a s e d   o n   t h e   i n d e x .   T h e   i n d e x   i s   u s e d   t o   r e t u r n   d i f f e r e n t   l i n e   t y p e  
 '   v a l u e s   o n   d i f f e r e n t   c a l l s   t o   G e t C h a r t R G B C o l o r .  
 '   D e f a u l t   i s   a   L i n e   t y p e   s t r i n g   t h a t   w i l l   b e   r e t u r n e d   o n c e   t h e   l i n e   t y p e s   r u n   o u t .  
 '   I f   D e f a u l t   i s   E m p t y ,   t h e   l i n e   t y p e s   w i l l   c y c l e   t h r o u g h   t h e   a v a i l a b l e   l i s t .  
 '   I n d e x   s h o u l d   b e   g r e a t e r   t h a n   0  
 '  
 F u n c t i o n   G e t C h a r t L i n e T y p e ( I n d e x ,   D e f a u l t )  
         L i n e T y p e s   =   A r r a y ( " D A S H " ,   " D O T " ,   " D A S H D O T " ,   " D A S H D O T D O T " ,   " S O L I D "   )  
         D i m   S e l e c t e d L i n e  
  
         I f   (   i n d e x   >   5   )   T h e n  
 	 I f   I s E m p t y   (   D e f a u l t   )     T h e n  
 	         S e l e c t e d L i n e   =   L i n e T y p e s   (   (   i n d e x   -   1   )   M o d   5   )  
 	 E l s e  
 	         S e l e c t e d L i n e   =   D e f a u l t  
 	 E n d   I f  
         E l s e  
 	 S e l e c t e d L i n e   =   L i n e T y p e s   (   i n d e x   -   1   )  
         E n d   I f  
  
         G e t C h a r t L i n e T y p e   =   S e l e c t e d L i n e  
 E n d   F u n c t i o n  
  
 '  
 '   F u n c t i o n   G e t C h a r t S y m b o l ( )  
 '   R e t u r n   a n   C h a r t   s y m b o l   b a s e d   o n   t h e   i n d e x .   T h e   i n d e x   i s   u s e d   t o   r e t u r n   d i f f e r e n t   s y m b o l    
 '   v a l u e s   o n   d i f f e r e n t   c a l l s   t o   G e t C h a r t R G B C o l o r .  
 '   D e f a u l t   i s   a   s y m b o l   s t r i n g   t h a t   w i l l   b e   r e t u r n e d   o n c e   t h e   s y m b o l s   r u n   o u t .  
 '   I f   D e f a u l t   i s   E m p t y ,   t h e   s y m b o l s   w i l l   c y c l e   t h r o u g h   t h e   a v a i l a b l e   l i s t .  
 '   I n d e x   s h o u l d   b e   g r e a t e r   t h a n   0  
 '  
 F u n c t i o n   G e t C h a r t S y m b o l ( I n d e x ,   D e f a u l t )  
         D i m   S e l e c t e d S y m b o l  
         S y m b o l s   =   A r r a y ( " S Y M _ C I R C L E " , " S Y M _ U P T R I A N G L E " , " S Y M _ D O W N T R I A N G L E " , " S Y M _ S Q U A R E " , " S Y M _ D I A M O N D " )  
  
         I f   (   i n d e x   >   5   )   T h e n  
 	 I f   I s E m p t y   (   D e f a u l t   )     T h e n  
 	         S e l e c t e d S y m b o l   =   S y m b o l s (   (   i n d e x   -   1   )   M o d   5 )  
 	 E l s e  
 	         S e l e c t e d S y m b o l   =   D e f a u l t  
 	 E n d   I f  
         E l s e  
 	 S e l e c t e d S y m b o l   =   S y m b o l s (   i n d e x   -   1   )  
         E n d   I f  
  
         G e t C h a r t S y m b o l   =   S e l e c t e d S y m b o l  
 E n d   F u n c t i o n  
  
 '    
 '   F u n c t i o n   G e t C h a r t C o m m a n d s ( )  
 '  
 '   R e t u r n s   a   c h a r t   c o m m a n d   s t r i n g   t h a t   c a n   b e   p a s s e d   t o   a   c h a r t   v i a   ! c m d   t o   d y n a m i c a l l y   d r a w   t r a c e s .  
 '   T h e   t r a c e s   w i l l   h a v e   d i f f e r e n t   c o l o r s   a n d   l i n e   t y p e s .  
 '  
 '   P a r a m e t e r s :  
 '  
 '   T r a c e N a m e :  
 '                       U s e d   t o   s e t   t h e   T r a c e   N a m e .   I f   m o r e   t h a n   o n e   t r a c e   i s   g e n e r a t e d ,   T h e   T r a c e   N a m e   w i l l  
 '                       h a v e   t h e   t r a c e   n u m b e r   a p p e n d e d   t o   i t .   E g :   T r a c e N a m e   =   " I m p e d e n c e " ,   t h e   t r a c e   n a m e s    
 '                       w i l l   b e   " I m p e d e n c e   1 " ,   " I m p e d e n c e   2 "   e t c .  
 '  
 '   S t a r t T r a c e N u m :  
 ' 	         T h e   t r a c e   n u m b e r   t o   s t a r t   a t .   E g :   I f   t h e   c h a r t   a l r e a d y   h a s   o n e   t r a c e   o n   i t ,   t r a c e  
 '                       n u m b e r   c a n   b e   s e t   t o   2   t o   a v o i d   o v e r w r i t i n g   t h a t   t r a c e .    
 '  
 '   X R a n g e :    
 '                       T h e   x   r a n g e   d e f i n i t i o n .   E g :   C e l l _ # ! 1 - N u m C e l l s    
 '  
 '   Y R a n g e :    
 '                       T h e   y   r a n g e   d e f i n i t i o n .   E g :   I m p e d e n c e # ! 1 - N u m C e l l s  
 '  
 '   N u m T r a c e s :  
 '                       T h e   n u m b e r   o f   t r a c e s   t o   g e n e r a t e .  
 '  
 '   S c a l e G r o u p :  
 '                       T h e   s c a l e   g r o u p   f o r   t h e   t r a c e .   ( s h o u l d   b e   1   o r   2 )  
 '  
 '   I n c l u d e S y m b o l s :  
 '                       W h e t h e r   s y m b o l s   s h o u l d   b e   i n c l u d e d   i n   t h e   t r a c e  
 '  
 ' N o t e s   o n   X R a n g e   a n d   Y R a n g e :    
 '       1 )   D o n ' t   u s e   t h e   :   c h a r a c t e r .  
 '       2 )   A n y   % T R A C E _ N U M %   i n   t h e   R a n g e   d e f i n i t i o n   s t r i n g s   w i l l   b e   r e p l a c e d   b y   t h e   t r a c e   n u m b e r .  
 '             e g :   I f   Y R a n g e =   " d b _ I m p e d e n c e _ # _ % T R A C E _ N U M % ! 1 - N u m C e l l s " ,     t h e   f i r s t   t r a c e   w i l l   h a v e   a  
 '             Y R a n g e   o f   " d b _ I m p e d e n c e _ # _ 1 ! 1 - N u m C e l l s "   a n d   t h e   s e c o n d   Y R a n g e   w i l l   h a v e    
 '             " d b _ I m p e d e n c e _ # _ 2 ! 1 - N u m C e l l s "   a n d   s o   o n .  
 '  
 ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' ' '  
 '   T r a c e   C o m m a n d   F o r m a t :  
 '    
 '   T h e   t r a c e   c o m m a n d   f o r m a t   a c c e p t e d   b y   c h a r t ! c m d   i s :  
 '  
 '   T R C = T r a c e   N u m b e r , T r a c e   N a m e , X   R a n g e , Y   R a n g e ,   R 2 5 5 ,   G 2 5 5 ,   B 1 2 2 ,   P o i n t   T y p e ,   L i n e   T y p e ,   S c a l e   G r o u p    
 '  
 '   X   R a n g e   f o r m a t :   $ V a r N a m e _ # ! S t a r t - E n d .     S t a r t   a n d   E n d   c a n   b e   c o n s t a n t s   o r   v a r i a b l e s .  
 '                                   T h e   d o l l a r   s i g n   p r e f i x   m e a n s   t h e   t a g   v a l u e s   w i l l   b e   s t r i n g s .  
 '  
 '   Y   R a n g e   f o r m a t :   S i m i l a r   t o   t h e   X   R a n g e   f o r m a t ,   b u t   $   i s   n o t   s u p p o r t e d .  
 '  
 '   P o i n t   T y p e :   o n e   o f   S Y M _ C I R C L E , S Y M _ N O N E , S Y M _ C I R C L E , S Y M _ U P T R I A N G L E , S Y M _ D O W N T R I A N G L E , S Y M _ S Q U A R E , S Y M _ D I A M O N D  
 '   L i n e   T y p e :   o n e   o f   S O L I D ,   D A S H D O T D O T ,   D A S H D O T ,   D O T ,   D A S H  
 '   S c a l e   G r o u p :   1   o r   2  
 '  
 F u n c t i o n   G e t C h a r t T r a c e C o m m a n d s ( T r a c e N a m e ,   S t a r t T r a c e N u m ,   X R a n g e ,   Y R a n g e ,   N u m T r a c e s ,   S c a l e G r o u p ,   I n c l u d e S y m b o l s )  
         C h a r t C m d   =   " "  
  
         T r a c e N u m   =   S t a r t T r a c e N u m  
          
         F o r   i   =   1   T o   N u m T r a c e s  
  
 	 C o l o r S t r   =   G e t C h a r t R G B C o l o r ( i )  
  
                 I f   I n c l u d e S y m b o l s   T h e n  
                         S y m b o l   =   G e t C h a r t S y m b o l ( i ,   E m p t y )  
                 E l s e  
             	         S y m b o l   =   " S Y M _ N O N E "  
                 E n d   I f  
 	  
 	 '   G o   t h r o u g h   l i n e   t y p e s   a n d   d e f a u l t   t o   " S O L I D "   o n c e   t h e   t y p e s   r u n   o u t .  
 	 L i n e T y p e   =   G e t C h a r t L i n e T y p e ( i ,   " S O L I D " )  
  
         T r a c e O f f s e t   =   G e t V a r ( " T r a c e O f f s e t " )  
 	 X S e r i e s   =   R e p l a c e ( X R a n g e ,   " % T R A C E _ N U M % " ,   C S t r ( i )   +   T r a c e O f f s e t )  
 	 Y S e r i e s   =   R e p l a c e ( Y R a n g e ,   " % T R A C E _ N U M % " ,   C S t r ( i )   +   T r a c e O f f s e t )  
  
 	 C h a r t C m d   =   C h a r t C m d   &   " T R C = "   &   T r a c e N u m   &   " , "   &   T r a c e N a m e   &   i   &   "   , "   &   X S e r i e s   &   " , "   &   Y S e r i e s   &   _  
 	                                         " , "   &   C o l o r S t r   &   " , "   &   S y m b o l   &   " , "   &   L i n e T y p e   &   " , "   &   S c a l e G r o u p   &   " : "  
  
 	 T r a c e N u m   =   T r a c e N u m   +   1  
         N e x t  
  
         G e t C h a r t T r a c e C o m m a n d s = C h a r t C m d 	      
 E n d   F u n c t i o n  
  
 s u b   P o s t O n U p d a t e ( )  
         D e f L i s t   =   G e t V a r ( " A u t o D e f i c i e n c y L i s t " )  
         i f   D e f L i s t   < >   E m p t y   t h e n    
                 D e f L i s t   =   " T e s t ( s )   "   +   D e f L i s t   +   "   F a i l e d "  
         e l s e    
                 D e f L i s t   =   "   "  
         e n d   i f    
  
     '     c a l l   S e t D e f i c i e n c i e s I n t e r n a l (   " C o m m e n t s 1 "   ,   D e f L i s t )  
 e n d   s u b    
 '  
 '   C a l c u l a t e   t h e   e r r o r   f o r   t h e   f i e l d   ' E r r T a g '    
 '   i f   t h e   e r r o r   i s   l a r g e r   t h a n   T o l e r a n c e ,   c o l o r   i t   r e d   a n d   a d d   i t   t o   t h e   d e f i c i a n c y   l i s t  
 '  
 s u b   C a l c E r r o r T a g (   E r r T a g ,   A c t u a l , D e s i r e d , T o l e r a n c e , D e f N o t e )  
         F g C o l o r   =   C L R _ B L A C K  
         E r r o r V a l   =   E m p t y  
         i f   I s N u m e r i c ( A c t u a l )   a n d   I s N u m e r i c ( D e s i r e d )   t h e n  
                 i f   n o t   I s E m p t y ( A c t u a l )   a n d   n o t   I s E m p t y ( D e s i r e d )   a n d   D e s i r e d < >   0   t h e n  
                         E r r o r V a l     =   a b s (   A c t u a l   -   D e s i r e d   )   /   D e s i r e d   *   1 0 0    
                         i f   E r r o r V a l   >   T o l e r a n c e   t h e n    
                                 C a l l   A d d F a i l u r e ( D e f N o t e )  
                                 F g C o l o r   =   C L R _ R E D  
                         e n d   i f    
                 e n d   i f    
         e n d   i f    
         c a l l   S e t V a r ( E r r T a g , E r r o r V a l )  
         c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   E r r T a g ,   " F g C o l o r       "   +   C S t r ( F g C o l o r )   )  
 e n d   s u b    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   C a l c u l a t e   t h e   e r r o r   f o r   t h e   f i e l d   ' E r r T a g '   b a s e d   o n   t h e   ' A c t u a l T a g '   a n d   ' D e s i r e d T a g '  
 '  
 '   i f   t h e   e r r o r   i s   l a r g e r   t h a n   T o l e r a n c e ,   c o l o r   i t   r e d   a n d   a d d   i t   t o   t h e   d e f i c i e n c y   l i s t   a n d   u s e   t h e    
 '   ' D e f N o t e '   t o   a n o t a t e   t h e   d e f i c i e n c y  
 '    
 '  
 '   D o   t h i s   f o r   t a g s   w i t h   a   s u f i x   o f   t h e   r a n g e   ' S t a r t N d x '   t o   ' E n d N d x '  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   C a l c E r r o r R a n g e (   E r r T a g ,   A c t u a l T a g , D e s i r e d T a g , T o l e r a n c e , D e f N o t e , S t a r t N d x ,   E n d N d x )  
          
         M s g   =   " "  
         b F a i l e d   =   f a l s e    
         f o r   N d x   =   S t a r t N d x   t o   E n d N d x  
                 F g C o l o r   =   C L R _ B L A C K  
                 E r r o r V a l   =   E m p t y  
                 A c t u a l   =   G e t N d x V a r ( A c t u a l T a g , N d x )  
                 D e s i r e d   =   G e t N d x V a r ( D e s i r e d T a g , N d x )  
                  
                 M s g   =   M s g   +   C S t r ( A c t u a l )   +   "     x x x   "   +   C S t r ( D e s i r e d )  
                  
                 i f   I s N u m e r i c ( A c t u a l )   a n d   I s N u m e r i c ( D e s i r e d )   t h e n  
                         i f   n o t   I s E m p t y ( A c t u a l )   a n d   n o t   I s E m p t y ( D e s i r e d )   a n d   D e s i r e d < >   0   t h e n  
                                 E r r o r V a l     =   a b s (   A c t u a l   -   D e s i r e d   )   /   D e s i r e d   *   1 0 0    
                                 i f   E r r o r V a l   >   T o l e r a n c e   t h e n    
                                         b F a i l e d   =   t r u e    
                                         F g C o l o r   =   C L R _ R E D  
                                 e n d   i f    
                         e n d   i f    
         e n d   i f    
               N d x E r r T a g   =   E r r T a g   +   " _ "   +   C S t r ( N d x )  
                 c a l l   S e t V a r ( N d x E r r T a g   , E r r o r V a l )  
                 c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   N d x E r r T a g   ,   " F g C o l o r       "   +   C S t r ( F g C o l o r )   )  
         n e x t    
         '  
         '   I f   a n y   o n e   o f   t h e   t e s t   f a i l ,   f l a g   t h e   o v e r a l l   t e s t   a s   a   f a i l u r e    
         '  
         i f   b F a i l e d   t h e n    
                 C a l l   A d d F a i l u r e ( D e f N o t e )  
         e n d   i f    
  
 e n d   s u b    
  
                          
                         '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   C a l c u l a t e   t h e   e r r o r   f o r   t h e   f i e l d   ' V a l u e '   b a s e d   o n   t h e   m i n   a n d   m a x   v a l u e s    
 '  
 '   i f   t h e   e r r o r   i s   l a r g e r   t h a n   T o l e r a n c e ,   c o l o r   i t   r e d   a n d   a d d   i t   t o   t h e   d e f i c i e n c y   l i s t   a n d   u s e   t h e    
 '   ' D e f N o t e '   t o   a n o t a t e   t h e   d e f i c i e n c y  
 '    
 '  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n     C h e c k E r r o r (   A c t u a l T a g ,   M i n V a l , M a x V a l , D e f N o t e )  
          
          
          
         C h e c k E r r o r   =   t r u e  
          
         M a x D e p t h   =   G e t V a r ( " F o r m M a x C a l c D e p t h " )  
         i f   I s E m p t y ( M a x D e p t h )   t h e n   M a x D e p t h   =   1  
         '  
         '   O n l y   d o   f o r   t h e   l a s t   e v a l u a t i o n   d e p t h  
         '  
         i f   F o r m . F o r m C u r C a l c D e p t h   =   M a x D e p t h     t h e n  
                  
                 M s g   =   " "  
                 F g C o l o r   =   C L R _ B L A C K  
  
                 A c t u a l   =   G e t V a r ( A c t u a l T a g )  
                                  
                 i f   I s N u m e r i c ( M i n V a l )   a n d   I s N u m e r i c ( M a x V a l )   a n d   I s N u m e r i c ( A c t u a l )   t h e n  
                         i f   n o t   I s E m p t y ( M i n V a l )   a n d   n o t   I s E m p t y ( M a x V a l )   a n d   n o t   I s E m p t y (   A c t u a l   )     t h e n  
                                 i f   A c t u a l   <   M i n V a l   o r   A c t u a l   >   M a x V a l   t h e n    
                                         C h e c k E r r o r   =   f a l s e    
                                         F g C o l o r   =   C L R _ R E D  
                                          
                                         i f   n o t   I s B l a n k S t r i n g (   D e f N o t e   )   t h e n    
                                                 C a l l   A d d F a i l u r e ( D e f N o t e )    
                                         e n d   i f    
                                 e n d   i f    
                         e n d   i f    
                 e n d   i f    
                 c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   A c t u a l T a g   ,   " F g C o l o r       "   +   C S t r ( F g C o l o r )   )  
       e n d   i f    
 e n d   f u n c t i o n    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   C a l c u l a t e   t h e   m i n i m u m   v a l u e   u s i n g   b o t h   a   p e r c e n t a g e   a n d   a n   a b s o l u t e   o f f s e t    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
  
 s u b   C a l c M i n M a x ( M i n T a g ,   M a x T a g ,   T h e o ,   P e r c e n t V a l ,   A b s V a l   )  
         C a l c M i n   =   e m p t y  
         C a l c M a x   =   e m p t y    
         i f   I s N u m e r i c (   T h e o )   a n d   I s N u m e r i c ( P e r c e n t V a l )   a n d   I s N u m e r i c ( A b s V a l )   t h e n  
                 i f   n o t   I s E m p t y ( T h e o )   a n d   n o t   I s E m p t y (   P e r c e n t V a l   )   a n d   N o t   i s E m p t y (   A b s V a l )   t h e n  
                         C a l c M i n   =   T h e o   *   (   1   -   ( P e r c e n t V a l   /   1 0 0 )   )   -   A b s V a l    
                         C a l c M a x   =   T h e o   *   (   1   + ( P e r c e n t V a l   /   1 0 0 )   )   +   A b s V a l    
                 e n d   i f    
         e n d   i f    
         c a l l   S e t V a r (   M i n T a g ,   C a l c M i n   )  
         c a l l   S e t V a r ( M a x T a g , C a l c M a x )  
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   C a l c u l a t e   t h e   m i n i m u m   v a l u e   u s i n g   b o t h   a   p e r c e n t a g e   a n d   a n   a b s o l u t e   o f f s e t   w i t h   p l u s   a n d   m i n u s  
 '   S e t   t h e   d e f   n o t e   a n d   c h a n g e   t h e   c o l o r   t o   r e d   i f   o u t   o f   t o l e r a n c e  
 '  
 '   R e t u r n s   t r u e   i f   i t   p a s s e d  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 f u n c t i o n   C a l c M i n M a x A n d E r r o r P m ( R e s u l t T a g ,   M i n T a g ,   M a x T a g ,   T h e o   ,   P e r c e n t P l u s , P e r c e n t M i n u s ,   A b s P l u s , A b s M i n u s ,   D e f N o t e   )  
  
         C a l c M i n   =   E m p t y  
         C a l c M a x   =   E m p t y  
          
         i f   I s E m p t y (   A b s P l u s   )   t h e n   A b s P l u s   =   0  
         i f   I s E m p t y (   P e r c e n t M i n u s   )   t h e n   P e r c e n t M i n u s   =   P e r c e n t P l u s  
         i f   I s E m p t y (   A b s M i n u s   )   t h e n   A b s M i n u s   =   A b s P l u s  
  
         i f   I s N u m e r i c (   T h e o )   a n d   I s N u m e r i c ( P e r c e n t M i n u s )   a n d   I s N u m e r i c ( A b s P l u s )   t h e n  
                 i f   n o t   I s E m p t y ( T h e o )   a n d   n o t   I s E m p t y (   P e r c e n t P l u s   )     t h e n  
                         C a l c M i n   =   T h e o   *   (   1   -   ( P e r c e n t M i n u s   /   1 0 0 )   )   -   A b s M i n u s    
                         C a l c M a x   =   T h e o   *   (   1   + ( P e r c e n t P l u s   /   1 0 0 )   )   +   A b s P l u s  
                 e n d   i f    
         e n d   i f    
         c a l l   S e t V a r (   M i n T a g ,   C a l c M i n   )  
         c a l l   S e t V a r ( M a x T a g , C a l c M a x )  
          
         C a l c M i n M a x A n d E r r o r P m   =   C h e c k E r r o r ( R e s u l t T a g ,   C a l c M i n ,   C a l c M a x ,   D e f N o t e )  
          
 e n d   f u n c t i o n  
  
  
  
 s u b   C a l c M i n M a x A n d E r r o r ( M i n T a g ,   M a x T a g ,   T h e o ,   P e r c e n t V a l ,   A b s V a l ,   A f T a g ,   A l T a g ,   D e f N o t e   )  
          
         c a l l   C a l c M i n M a x ( M i n T a g ,   M a x T a g ,   T h e o ,   P e r c e n t V a l ,   A b s V a l   )  
         c a l l   C h e c k E r r o r ( A f T a g ,   G e t V a r ( M i n T a g ) ,   G e t V a r ( M a x T a g ) ,   " " )  
         c a l l   C h e c k E r r o r ( A l T a g ,   G e t V a r ( M i n T a g ) ,   G e t V a r ( M a x T a g ) ,   D e f N o t e )          
          
 e n d   s u b  
  
  
  
 s u b   C a l c E r r o r A b s R a n g e (   E r r T a g ,   A c t u a l T a g , D e s i r e d T a g , T o l e r a n c e ,   A b s T o l e r a n c e ,   D e f N o t e , S t a r t N d x ,   E n d N d x )  
          
         M s g   =   " "  
         b F a i l e d   =   f a l s e    
         f o r   N d x   =   S t a r t N d x   t o   E n d N d x  
                 F g C o l o r   =   C L R _ B L A C K  
                 E r r o r V a l   =   E m p t y  
                 A c t u a l   =   G e t N d x V a r ( A c t u a l T a g , N d x )  
                 D e s i r e d   =   G e t N d x V a r ( D e s i r e d T a g , N d x )  
                  
                   c a l l   C a l c M i n M a x ( " T m p M i n " ,   " T m p M a x " ,   D e s i r e d ,   T o l e r a n c e ,   A b s T o l e r a n c e   )  
                    
                   T m p M i n   =   G e t V a r ( " T m p M i n " )  
                   T m p M a x   =   G e t V a r ( " T m p M a x " )  
                  
                 M s g   =   M s g   +   C S t r ( A c t u a l )   +   "     x x x   "   +   C S t r ( D e s i r e d )  
                  
                 i f   I s N u m e r i c ( A c t u a l )   a n d   I s N u m e r i c ( D e s i r e d )   t h e n  
                         i f   n o t   I s E m p t y ( A c t u a l )   a n d   n o t   I s E m p t y ( D e s i r e d )   a n d   D e s i r e d < >   0   t h e n  
                                 E r r o r V a l     =   a b s (   A c t u a l   -   D e s i r e d   )   /   D e s i r e d   *   1 0 0    
                                 i f   A c t u a l   >   T m p M a x   o r   A c t u a l   <   T m p M i n   t h e n    
                                         b F a i l e d   =   t r u e    
                                         F g C o l o r   =   C L R _ R E D  
                                 e n d   i f    
                         e n d   i f    
         e n d   i f    
               N d x E r r T a g   =   E r r T a g   +   " _ "   +   C S t r ( N d x )  
                 c a l l   S e t V a r ( N d x E r r T a g   , E r r o r V a l )  
                 c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   N d x E r r T a g   ,   " F g C o l o r       "   +   C S t r ( F g C o l o r )   )  
         n e x t    
         '  
         '   I f   a n y   o n e   o f   t h e   t e s t   f a i l ,   f l a g   t h e   o v e r a l l   t e s t   a s   a   f a i l u r e    
         '  
         i f   b F a i l e d   t h e n    
                 C a l l   A d d F a i l u r e ( D e f N o t e )  
         e n d   i f    
  
 e n d   s u b    
  
  
 f u n c t i o n   M a k e S t r F o r R u n T a g C m d ( T a g N a m e )  
  
         R e t V a l   =   G e t V a r ( T a g N a m e )  
         R e t V a l   =   R e p l a c e ( R e t V a l ,   " ' " ,   " ' ' " )  
          
         M a k e S t r F o r R u n T a g C m d   =   " ' "   +   R e t V a l   +   " ' "  
          
 e n d   f u n c t i o n  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 '   F U N C T I O N   G e t N d x V a r M i d E x ( S t r A , N d x , S t r B )  
 '  
 '     T h i s   f u n c t i o n   r e t u r n s   t h e   v a l u e   o f   a   v a r i a b l e   w i t h   a n d   i n d e x   i n   t h e   m i d d l e  
 '  
 '     I f   t h e   _ 1 _   d o e s   n o t   e x i s t ,   c h e c k   _   ( f o r   s i n g l e   r e c o r d   d o b l e   v a l u e s )    
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   G e t N d x V a r M i d E x ( S t r A I n , N d x , S t r B )  
         S t r a   =   S t r A I n  
         G e t N d x V a r M i d E x   =   " "  
         N a m e   =   S t r A   +   C S t r ( N d x )   +   S t r B  
         G e t N d x V a r M i d E x     =   G e t V a r ( N a m e )  
         i f   G e t N d x V a r M i d E x   =   E m p t y   a n d   N d x   =   1   t h e n    
                 L e n A   =   L e n ( S t r A )  
                 i f   L e n A   >   2   t h e n    
                         S t r a   =   L e f t ( S t r a , L e n A - 1 )  
                         N a m e   =   S t r A   +   S t r B  
 c a l l   a l o g ( " D o u b l e   c h e c k   "   +   N a m e )                          
                         G e t N d x V a r M i d E x     =   G e t V a r ( N a m e )  
                 e n d   i f    
         e n d   i f    
 e n d   f u n c t i o n  
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   A d d F a i l u r e ( )  
 '   A d d   a   f a i l u r e   n o t e   t o   t h e   a u t o   d e f i c i e n c i e s   s e c t i o n  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 S U B   A d d F a i l u r e ( D e f N o t e )            
         D e f L i s t   =   G e t V a r ( " A u t o D e f i c i e n c y L i s t " )                                
         M a x D e p t h   =   G e t V a r ( " F o r m M a x C a l c D e p t h " )  
         i f   I s E m p t y ( M a x D e p t h )   t h e n   M a x D e p t h   =   1  
         i f   D e f L i s t   < >   E m p t y   t h e n   D e f L i s t   =   D e f L i s t   +   " , "            
         D e f L i s t   =   D e f L i s t   +   D e f N o t e                                                        
         i f   F o r m . F o r m C u r C a l c D e p t h   =   M a x D e p t h - 1     t h e n                  
                 C a l l   S e t V a r ( " A u t o D e f i c i e n c y L i s t " , D e f L i s t )                        
         e n d   i f    
 E N D   S U B                              
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t H i s t o r y D a t a ( )  
 '   G e t   t h e   h i s t o r i c a l   d a t a   f o r   t h e   t a g s   i n t o   t h e   v a r i a b l e s  
 '   d b _ < v a r i a b l e N a m e > _ # .     I t   w i l l   a l s o   u p d a t e   t h e    
 '   d b _ < v a r i a b l e N a m e > _ 0   r e c o r d   w i t h   t h e   t a g   v a l u e    
 '   t o   p l o t   t h e   p r e s e n t   r e s u l t s   w i t h   t h e   h i s t o r i c a l   v a l u e s  
 '  
 '   i t   a l s o   u p d a t e s . .    
 '         d b _ < V a r i a b l e N a m e > _ A v g  
 '         d b _ < V a r i a b l e N a m e > _ M i n  
 '         d b _ < V a r i a b l e N a m e > _ M a x  
 '         d b _ < V a r i a b l e N a m e > _ S t d  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   G e t H i s t o r y D a t a (   T a g s   )  
          
         i f   F o r m . F o r m L o a d e d V a l u e s   a n d   G e t V a r ( " _ _ H i s t o r y L o a d e d " )   < >   1   t h e n  
                 c a l l   s e t V a r ( " _ _ H i s t o r y L o a d e d " , 1 )  
                 '    
                 '   N u l l   a n y   o l d   r e c o r d s  
                 '    
                 D i m   p a r t s ( 1 0 0 )  
                 c o u n t   =   P a r s e S t r i n g ( T a g s ,   p a r t s ,   " , " ,   1 0 0 )  
                  
                 f o r   i   =   1   t o   C o u n t  
                         f o r   H i s t o r y   =   1   t o   1 0    
                                 c a l l   S e t N d x V a r ( " d b _ "   +   p a r t s ( i ) , H i s t o r y , E m p t y )  
                         n e x t    
                 n e x t  
                  
                 c a l l   a p p l i c a t i o n . G e t H i s t D a t a F o r T a g s ( f o r m . f o r m i d , T a g s )  
                  
         e n d   i f    
  
         '  
         '   U p d a t e   a n y   n e w l y   e n t e r e d   t a g s   t o   t h e   d b _ < T a g N a m e > _ 0    
         '   v a r i a b l e    
         c a l l   U p d a t e D b Z e r o ( T a g s )  
  
  
  
 e n d   s u b    
                                                                          
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e D b Z e r o ( )  
 '   U p d a t e s   t h e   d b _ < v a r i a b l e N a m e > _ 0   r e c o r d s   w i t h    
 '   t h e   t a g   v a l u e s   t o   p l o t   t h e   p r e s e n t   r e s u l t s   w i t h   t h e    
 '   h i s t o r i c a l   v a l u e s  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 s u b   U p d a t e D b Z e r o (   T a g s   )  
                 D i m   p a r t s ( 1 0 0 )  
                 c o u n t   =   P a r s e S t r i n g ( T a g s ,   p a r t s ,   " , " ,   1 0 0 )  
                  
                 c a l l   a l o g ( " C o u n t   =   "   +   C S t r ( C o u n t )   )  
                 f o r   i   =   1   t o   C o u n t  
                         T a g   =   P a r t s ( i )  
                         V a l u e   =   G e t V a r ( T a g )  
                         c a l l   S e t N d x V a r ( " d b _ "   +   T a g , 0 , V a l u e )  
                 n e x t  
 e n d   s u b    
  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   H H M M t o S e c s ( )  
 '   C o n v e r t   a   H H : M M   s t r i n g   t o   s e c o n d s .  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
  
 f u n c t i o n   H H M M t o S e c s ( T i m e S t r )  
         d i m   p a r t s ( 2 )  
         V a l u e   =   0  
         c o u n t   =   P a r s e S t r i n g ( T i m e S t r ,   p a r t s ,   " : " ,   2 )          
         f o r   i   =   1   t o   c o u n t  
                 V a l u e   =   V a l u e   +     T o N u m b e r (   p a r t s ( i )   )  
                 V a l u e   =   V a l u e   *   6 0  
         n e x t  
         h h M M t o S e c s   =   V a l u e  
 e n d   f u n c t i o n  
  
        
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F U N C T I O N   S e c s T o H H M M S S (   T i m e I n S e c o n d s   )  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
          
         T i m e I n S e c o n d s   =     C L n g (   T o N u m b e r (   T i m e I n S e c o n d s   )   )  
         H H   =   I n t (   D i v i d e (   T i m e I n S e c o n d s ,   3 6 0 0   )   )  
         M M   =   T i m e I n S e c o n d s   M o d   3 6 0 0  
         S S   =   M M   M o d   6 0  
         M M   =   I n t (   D i v i d e (   M M ,   6 0   )   )  
          
         S e c s T o H H M M S S   =   r i g h t ( " 0 " &   H H , 2 )   &   " : "   &   r i g h t ( " 0 " &   M M , 2 )   &   " : "   &   r i g h t ( " 0 " &   S S , 2 )  
          
 E N D   F U N C T I O N  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F U N C T I O N   F o r m a t T i m e H H M M S S (   T i m e S t r   )  
 '       C o n v e r t s   a   t i m e   s t r i n g   i n t o   H H : M M : S S   f o r m a t  
 '       I f   t h e   i n p u t   s t r i n g   c o n t a i n s   c o l o n ( s ) ,   i t   i s   t r e a t e d   a s   H H : M M ( : S S )   f o r m a t  
 '       O t h e r w i s e ,   t h e   i n p u t   i s   a s s u m e d   t o   b e   a   v a l u e   i n   s e c o n d s  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
         i f   (   I n S t r (   T i m e S t r ,   " : "   )   )   t h e n  
                  
                 H H S t r   =   L e f t (   T i m e S t r ,   I n S t r (   T i m e S t r ,   " : "   )   -   1   )  
                 T i m e S t r   =   M i d (   T i m e S t r ,   I n S t r (   T i m e S t r ,   " : "   )   +   1   )  
                  
                 i f (   I n S t r (   T i m e S t r ,   " : "   )   )   t h e n  
                         M M S t r   =   L e f t (   T i m e S t r ,   I n S t r (   T i m e S t r ,   " : " )   -   1   )  
                         S S S t r   =   M i d (   T i m e S t r ,   I n S t r (   T i m e S t r ,   " : "   )   +   1   )  
                 e l s e  
                         M M S t r   =   T i m e S t r  
                         S S S t r   =   " 0 0 "  
                 e n d   i f  
  
                 H H   =   I n t (   T o N u m b e r (   H H S t r   )   )  
                 M M   =   I n t (   T o N u m b e r (   M M S t r   )   )  
                 S S   =   I n t (   T o N u m b e r (   S S S t r   )   )  
                                  
                 i f (   S S   >   5 9   )   t h e n  
                         M M   =   M M   +   C I n t (   D i v i d e (   S S ,   6 0   )   )  
                         S S   =   S S   M o d   6 0  
                 e n d   i f  
                 i f (   S S   <   0   )   t h e n  
                         S S   =   0  
                 e n d   i f  
  
                 i f (   M M   >   5 9   )   t h e n  
                         H H   =   H H   +   C I n t (   D i v i d e (   M M ,   6 0   )   )  
                         M M   =   M M   M o d   6 0  
                 e n d   i f  
                 i f (   M M   <   0   )   t h e n  
                         M M   =   0  
                 e n d   i f  
          
                 i f (   H H   >   9 9   )   t h e n  
                         H H   =   9 9  
                         M M   =   5 9  
                         S S   =   5 9  
                 e n d   i f  
                 i f (   H H   <   0   )   t h e n  
                         H H   =   0  
                 e n d   i f      
              
                 H H S t r   =   r i g h t ( " 0 " &   H H , 2 )  
                 M M S t r   =   r i g h t ( " 0 " &   M M , 2 )  
                 S S S t r   =   r i g h t ( " 0 " &   S S , 2 )  
                          
                 F o r m a t T i m e H H M M S S   =   H H S t r   &   " : "   &   M M S t r   &   " : "   &   S S S t r  
  
         e l s e  
                  
                 F o r m a t T i m e H H M M S S   =   S e c s T o H H M M S S (   T i m e S t r   )  
                  
         e n d   i f  
  
 E N D   F U N C T I O N  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 '   S U B   A d d D e f i c i e n c y ( )  
 '  
 '   T h i s   m e t h o d   a d d s   a   d e f i c i e n c y   n o t e   t o   t h e   a u t o   d e f i c i e n c y   l i n e .  
 '   T h e   n o t e   i s   a d d e d   u n c h a n g e d ,   a n d   n o   " F a i l e d "   t e x t   i s   a p p e n d e d   a s   i s   d o n e  
 '   b y   A d d F a i l u r e ( ) .  
 '  
 '   T e m p o r a r i l y   a d d e d   t o   t h i s   t e m p l a t e ,   t o   s u p p o r t   o l d e r   v e r s i o n s   t h a t   d o - n o t   h a v e   t h i s  
 '   c h a n g e   i n   S t a n d a r d S c r i p t s . h  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S U B   A d d D e f i c i e n c y ( D e f N o t e )  
         D e f L i s t   =   G e t V a r ( " D i r e c t D e f i c i e n c y L i s t " )  
         i f   D e f L i s t   < >   E m p t y   t h e n   D e f L i s t   =   D e f L i s t   &   " , \ n "  
                  
         D e f L i s t   =   D e f L i s t   &   D e f N o t e  
    
         M a x D e p t h   =   G e t V a r ( " F o r m M a x C a l c D e p t h " )  
         i f   I s E m p t y ( M a x D e p t h )   t h e n   M a x D e p t h   =   1        
         i f   F o r m . F o r m C u r C a l c D e p t h   =   M a x D e p t h - 1   t h e n  
                   C a l l   S e t V a r (   " D i r e c t D e f i c i e n c y L i s t " , D e f L i s t )  
         E n d   i f  
 E N D   S U B  
  
 F U N C T I O N   H a v e A u t o D e f s ( )  
         H a v e   =   f a l s e          
  
                 P d b A u t o D e f s   =   G e t V a r ( " P d b A u t o D e f s " )  
                 A u t o D e f i c i e n c y L i s t   =   G e t V a r ( " A u t o D e f i c i e n c y L i s t " )  
                 D i r e c t D e f i c i e n c y L i s t   =   G e t V a r ( " D i r e c t D e f i c i e n c y L i s t " )  
               '   C a l l   a l o g ( " I n   H a v e A u t o   1 :   "   &   A u t o D e f i c i e n c y L i s t )  
               '   C a l l   a l o g ( " I n   H a v e A u t o   2 :   "   &   P d b A u t o D e f s )  
               '   C a l l   a l o g ( " I n   H a v e A u t o   3 :   "   &   D i r e c t D e f i c i e n c y L i s t )  
                 I f   N o t   I s B l a n k S t r i n g ( P d b A u t o D e f s )   O R   N o t   I s B l a n k S t r i n g ( A u t o D e f i c i e n c y L i s t )   O R   N o t   I s B l a n k S t r i n g ( D i r e c t D e f i c i e n c y L i s t )   T h e n  
                         H a v e   =   t r u e  
                 E n d   I f  
                  
         H a v e A u t o D e f s   =   H a v e  
            
 E N D   F U N C T I O N  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 '   S U B   P o s t O n U p d a t e 2 ( )  
 '  
 '   O v e r r i d e s   t h e   d e f a u l t   P o s t O n U p d a t e 2 ( )   b y   a d d i n g   s u p p o r t   f o r   " D i r e c t D e f i c i e n c y L i s t "  
 '  
 '   T e m p o r a r i l y   a d d e d   t o   t h i s   t e m p l a t e ,   t o   s u p p o r t   o l d e r   v e r s i o n s   t h a t   d o - n o t   h a v e   t h i s  
 '   c h a n g e   i n   S t a n d a r d S c r i p t s . h  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S U B   P o s t O n U p d a t e 2 N o t A n y m o r e ( )  
       D i s p I d   =   F o r m . G e t D i s p I d ( " A u t o D e f i c i e n c y _ 1 " )  
       i f   D i s p I d   >   0   t h e n  
           D e f L i s t   =   G e t V a r ( " A u t o D e f i c i e n c y L i s t " )  
           D e f L i s t 2   =   G e t V a r ( " D e f L i s t " )  
            
           i f   D e f L i s t   < >   E m p t y   a n d   D e f L i s t 2   < >   E m p t y   t h e n   D e f L i s t   =   D e f L i s t   &   " , \ n "  
           D e f L i s t   =     D e f L i s t   &   D e f L i s t 2  
  
           I f   D e f L i s t   < >   E m p t y     T h e n  
                   D e f L i s t   =   D e f L i s t   +   T L a n g ( "   F a i l e d " )  
           E n d   I f  
            
           D e f L i s t 3   =   G e t V a r ( " D i r e c t D e f i c i e n c y L i s t " )            
           i f   D e f L i s t   < >   E m p t y     a n d   D e f L i s t 3   < >   E m p t y   t h e n   D e f L i s t   =   D e f L i s t   &   " , \ n "  
           D e f L i s t   =     D e f L i s t   &   D e f L i s t 3  
            
           D e f L i s t 4   =   G e t V a r ( " P d b A u t o D e f s " )            
           i f   D e f L i s t   < >   E m p t y     a n d   D e f L i s t 4   < >   E m p t y   t h e n   D e f L i s t   =   D e f L i s t   &   " , \ n "  
           D e f L i s t   =     D e f L i s t   &   D e f L i s t 4  
            
           P d b V e r s i o n   =   G e t V e r s i o n ( )    
            
           I f   D e f L i s t   < >   E m p t y   T h e n  
                   I f   P d b V e r s i o n   > =   8 0 0 0 . 2 7   t h e n                            
                           C a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   " A u t o D e f i c i e n c y _ 1 " ,   " W r a p T e x t T o T a b l e   ' "   &   D e f L i s t   &   " ' "   )  
                   E l s e  
                           C a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   " A u t o D e f i c i e n c y _ 1 " ,   " S e t W r a p p e d T e x t   ' "   &   D e f L i s t   &   " ' "   )                    
                   E n d   I f                
                      
         E n d   I f              
    
              
         i f   D e f L i s t   =     E m p t y   t h e n   c a l l   S e t V a r (   " A u t o D e f i c i e n c y _ 1 " ,   E m p t y   )  
              
 e l s e  
               C a l l   P o s t O n U p d a t e ( )  
       e n d   i f  
       ' c a l l   a l o g (   " T e s t i n g   D i s p I d   =   "   &   D i s p I d   &     "     d e f l i s t   =   "   &   D e f L i s t   )  
 E N D   S U B  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 '   S U B   P r e O n U p d a t e ( )  
 '  
 '   O v e r r i d e s   t h e   d e f a u l t   P r e O n U p d a t e ( )   b y   a d d i n g   s u p p o r t   f o r   " D i r e c t D e f i c i e n c y L i s t "  
 '  
 '   T e m p o r a r i l y   a d d e d   t o   t h i s   t e m p l a t e ,   t o   s u p p o r t   o l d e r   v e r s i o n s   t h a t   d o - n o t   h a v e   t h i s  
 '   c h a n g e   i n   S t a n d a r d S c r i p t s . h  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S U B   P r e O n U p d a t e ( )  
         c a l l   S e t V a r ( " A u t o D e f i c i e n c y L i s t " , E m p t y )  
         c a l l   S e t V a r ( " D i r e c t D e f i c i e n c y L i s t " , E m p t y )  
 E N D   S U B  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '       F U N C T I O N   G e t M i n ( )  
 '            
 '       R e t u r n s   t h e   l e s s e r   o f   t w o   n u m b e r s  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F U N C T I O N   G e t M i n (   N u m 1 ,   N u m 2   )  
          
         i f (   N u m 1   < =   N u m 2   )   t h e n  
                 G e t M i n   =   N u m 1  
         e l s e  
                 G e t M i n   =   N u m 2  
         e n d   i f  
          
 E N D   F U N C T I O N  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '       F U N C T I O N   G e t M a x ( )  
 '            
 '       R e t u r n s   t h e   g r e a t e r   o f   t w o   n u m b e r s  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F U N C T I O N   G e t M a x (   N u m 1 ,   N u m 2   )  
         i f (   N u m 1   > =   N u m 2   )   t h e n  
                 G e t M a x   =   N u m 1  
         e l s e  
                 G e t M a x   =   N u m 2  
         e n d   i f  
                    
 E N D   F U N C T I O N  
  
 F U N C T I O N   G e t H d r T e m p U n i t ( )  
  
     b F a h r e n h e i t   =   t r u e  
  
     '  
     '   C h e c k   t h e   o l d   s t r i n g   u n i t   i f   i t   d e f i n e d ,   o t h e r w i s e   u s e   t h e   n e w   i n t e g e r   u n i t .  
     '    
     h t u S t r   =   G e t V a r ( " H d r T e m p U n i t " )  
     i f   I s E m p t y ( h t u S t r )   t h e n  
         i f   ( G e t V a r ( " H d r T e m p U n i t I n t " )   =   G e t V a r ( " H d r T e m p U n i t I n t C " ) )   t h e n  
             b F a h r e n h e i t   =   f a l s e  
         e n d   i f  
     e l s e  
         i f   I n S t r ( h t u S t r ,   " C " )   t h e n    
             b F a h r e n h e i t   =   f a l s e  
         e n d   i f  
     e n d   i f  
  
     i f   b F a h r e n h e i t   t h e n  
         G e t H d r T e m p U n i t   =   " F "  
     e l s e  
         G e t H d r T e m p U n i t   =   " C "  
     e n d   i f  
  
 E N D   F U N C T I O N  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S e t T e m p e r a t u r e C  
 '  
 '   C h a n g e s   t e m p e r a t u r e   t o   t h e   g i v e n   v a l u e   ( i n   c )  
 '  
 '   H a n d l e s   c o n v e r s i o n   t o   F   i f   n e e d e d  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   S e t T e m p e r a t u r e C ( T e m p )  
  
         D i m   T e m p F ,   T e m p C ,   T e m p U n i t  
                
         T e m p F   =   F o r m a t N u m b e r (   C t o F (   T e m p   ) , 1 )  
         T e m p C   =   T e m p                  
                  
         c a l l   S e t V a r ( " T e m p e r a t u r e " ,   T e m p F )  
         c a l l   S e t V a r ( " T e m p e r a t u r e C " ,   T e m p C )  
  
         T e m p U n i t   =   G e t H d r T e m p U n i t ( )  
                  
         i f   I n S t r ( T e m p U n i t ,   " C " )   t h e n    
             c a l l   S e t V a r ( " E n t e r T e m p " ,   T e m p C )  
         e l s e  
             c a l l   S e t V a r ( " E n t e r T e m p " ,   T e m p F )  
         e n d   i f  
          
 e n d   s u b  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S e t T e m p e r a t u r e F  
 '  
 '   C h a n g e s   t e m p e r a t u r e   t o   t h e   g i v e n   v a l u e   ( i n   F )  
 '  
 '   H a n d l e s   c o n v e r s i o n   t o   C   i f   n e e d e d  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   S e t T e m p e r a t u r e F ( T e m p )  
          
         D i m   T e m p F ,   T e m p C ,   T e m p U n i t  
  
         T e m p F   =   T e m p  
         T e m p C   =   F o r m a t N u m b e r (   F t o C (   T e m p   ) , 1 )  
                  
         c a l l   S e t V a r ( " T e m p e r a t u r e " ,   T e m p F )  
         c a l l   S e t V a r ( " T e m p e r a t u r e C " ,   T e m p C )  
  
         T e m p U n i t   =   G e t H d r T e m p U n i t ( )  
          
         i f   I n S t r (   T e m p U n i t   ,   " C "   )   t h e n    
                 c a l l   S e t V a r ( " E n t e r T e m p " ,   T e m p C )  
         e l s e  
                 c a l l   S e t V a r ( " E n t e r T e m p " ,   T e m p F )  
         e n d   i f  
          
 e n d   s u b  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t T e m p e r a t u r e C  
 '  
 '   G e t   t h e   t e m p e r a t u r e   i n   C  
 '  
 '   H a n d l e s   c o n v e r s i o n   f r o m   F   i f   n e e d e d  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   G e t T e m p e r a t u r e C ( )  
  
         D i m   T e m p C ,   T e m p U n i t  
                      
         T e m p U n i t   =   G e t H d r T e m p U n i t ( )  
         E n t e r T e m p   =   G e t V a r ( " E n t e r T e m p " )  
          
         i f   I s B l a n k S t r i n g ( T e m p U n i t )   A n d   I s B l a n k S t r i n g ( E n t e r T e m p )   T h e n  
                  
                 T e m p C   =   F t o C (   G e t V a r ( " T e m p e r a t u r e " )   )                          
                  
         e l s e i f   I n S t r (   T e m p U n i t   ,   " C "   )   t h e n  
                 T e m p C   =   E n t e r T e m p  
         e l s e  
                 T e m p C   =   F t o C (   E n t e r T e m p   )  
         e n d   i f  
          
         G e t T e m p e r a t u r e C   =   T e m p C  
          
 e n d   f u n c t i o n  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t T e m p e r a t u r e F  
 '  
 '   G e t   t e m p e r a t u r e   v a l u e   i n   F  
 '  
 '   H a n d l e s   c o n v e r s i o n   f r o m   C   i f   n e e d e d  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   G e t T e m p e r a t u r e F ( )  
  
         D i m   T e m p F ,   T e m p U n i t  
                      
         T e m p U n i t   =   G e t H d r T e m p U n i t ( )  
         E n t e r T e m p   =   G e t V a r ( " E n t e r T e m p " )  
          
         i f   I s B l a n k S t r i n g ( T e m p U n i t )   A n d   I s B l a n k S t r i n g ( E n t e r T e m p )   T h e n  
                  
                 T e m p F   =   G e t V a r ( " T e m p e r a t u r e " )  
                  
         e l s e i f   I n S t r (   T e m p U n i t   ,   " C "   )   t h e n  
                 T e m p F   =   C t o F (   E n t e r T e m p   )  
         e l s e  
                 T e m p F   =   E n t e r T e m p  
         e n d   i f  
          
 e n d   f u n c t i o n  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '  
 '   D o M o d a l F o r m W i t h R e t u r n  
 '  
 '     L o a d s   a   f o r m   i n   a   m o d a l   w i n d o w ,   t h e n   p a u s e s   s c r i p t   u n t i l   t h e   c h i l d   f o r m   c l o s e s  
 '  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 S U B   D o M o d a l F o r m W i t h R e t u r n (   F o r m N a m e   )  
  
         P a r e n t F o r m I d   =   G e t V a r (   " F o r m I d "   )  
  
         c a l l   a p p l i c a t i o n . d o M o d a l F o r m (   F o r m N a m e   )  
  
         d o  
               s l e e p ( 1 )  
         L o o p   w h i l e   ( G e t V a r ( " F o r m I d " )   =   P a r e n t F o r m I d )  
  
         d o  
               s l e e p ( 1 )              
         L o o p   w h i l e   ( G e t V a r ( " F o r m I d " )   < >   P a r e n t F o r m I d )  
  
 E N D   S U B    
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S u b   M a p T a g s ( T a g M a p )  
 '  
 '   M a p s   e x i s t i n g   t a g s   t o   n e w   t a g   n a m e s   b a s e d   o n   p a s s e d   t a g   m a p .  
 '  
 '   P a r a m e t e r s :  
 '             T a g M a p :   N a m e   o f   t a g   m a p   a v a i l a b l e   i n   d a t a   t a g .   E g :   " T e g g T a g M a p : "  
 '                             N e e d s   t o   i n c l u d e   t h e   c o l o n   ( : ) .  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S u b   M a p T a g s ( T a g M a p )  
  
         D i m   L i n e s ,   S t a r t N d x ,   E n d N d x ,   S r c T a g ,   D e s t T a g ,   P a r t s ,   S r c V a l  
          
  
         T e g T a g M a p   =   A p p l i c a t i o n . R e a d D a t a M u l t i L i n e F r o m T a g ( F o r m . F o r m I d ,   T a g M a p )  
         L i n e s   =   S p l i t ( T e g T a g M a p ,   v b L f )  
  
         S t a r t N d x   =   L B o u n d ( L i n e s )  
         E n d N d x   =   U B o u n d ( L i n e s )  
  
         F o r   N d x   =   S t a r t N d x   T o   E n d N d x  
  
                 P a r t s   =   S p l i t ( L i n e s ( N d x ) ,   " , " )  
                 I f   U B o u n d ( P a r t s )   > =   1   T h e n  
                         S r c T a g   =   T r i m ( P a r t s ( 0 ) )  
                         D e s t T a g   =   T r i m ( P a r t s ( 1 ) )  
                          
                         I f   N o t   I s B l a n k S t r i n g ( S r c T a g )   a n d   N o t   I s B l a n k S t r i n g ( D e s t T a g )   T h e n  
                                 S r c V a l   =   G e t V a r ( S r c T a g )  
                                 ' C a l l   a L o g ( " S r c V a l   =   "   &   S r c V a l   &   " ,   D e s t T a g   =   "   &   D e s t T a g   &   " ,   S r c T a g   =   "   &   S r c T a g )  
                                 C a l l   S e t V a r ( D e s t T a g ,   S r c V a l )  
                                  
                         E n d   I f                          
                          
                 E n d   I f  
         N e x t  
  
 E n d   S u b  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S u b   T o g g l e ( )  
 '       S w a p   T a g n a m e   v a l u e   b e t w e e n   1   a n d   0  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   T o g g l e ( T a g )  
         v a l   =   G e t V a r ( T a g )  
         i f   v a l   t h e n    
                 v a l   =   0  
         e l s e  
                 v a l   =   1  
         e n d   i f    
         c a l l   S e t V a r ( T a g , v a l )  
 e n d   s u b    
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S u b   S i n k ( )  
 '       D i s p l a y   a   b u t t o n   a s   s u n k e n  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 s u b   S i n k (   T a g s   ,   S t a t e   )  
          
         i f   S t a t e   t h e n    
                 C m d   =   " S i n k   2   ,   "   &   C L R _ Y E L L O W  
         e l s e  
                 C m d   =   " S i n k   1 ,   "   &   C L R _ G R A Y 2 0  
         e n d   i f    
         c a l l   R u n T a g C m d ( T a g s , C m d )  
 e n d   s u b    
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   f u n c t i o n   G e t R o w F r o m T a g ( )  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   G e t R o w F r o m T a g ( T a g N a m e )  
     G e t R o w F r o m T a g   =   0  
     L e n g t h   =   L e n ( T a g N a m e )  
     i f   L e n g t h   >   0   t h e n  
         f o r   n d x   =   1   t o   L e n g t h  
                 i f   M i d ( T a g N a m e , N d x , 1 )   =   " _ "   t h e n  
                         U n d e r S c o r e N d x   =   N d x  
                 e n d   i f  
         n e x t  
     e n d   i f  
      
     i f   U n d e r S c o r e N d x   >   0   a n d   U n d e r S c o r e N d x   <   L e n g t h   t h e n  
 	 R e s t O f S t r   =   M i d (   T a g N a m e , U n d e r S c o r e N d x + 1 )  
         R o w N d x     =   T o N u m b e r (   R e s t O f S t r   )  
         i f   R o w N d x   <   1   t h e n   R o w N d x   =   1  
         G e t R o w F r o m T a g   =   R o w N d x  
     e n d   i f  
 e n d   f u n c t i o n  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   f u n c t i o n   a c o s ( a )  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 f u n c t i o n   A c o s ( a )  
       i f   A b s ( a ) = 1   t h e n    
               A c o s   =   ( 1 - a ) * P I / 2  
       e l s e    
               a l o g ( " a   =   "   &   a   )  
               A c o s   =   A t n ( - a / s q r (   1 - a * a   )   ) + 2 * A t n ( 1 )  
         e n d   i f  
 e n d   f u n c t i o n  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   f u n c t i o n   P o l a r D e g r e e s T o C a r t e s i a n  
 '       C o n v e r t s   p o l a r   c o o r d s   t o   c a r t e s i a n  
 '       R e t u r n s   t r u e   i f   c o n v e r s i o n   i s   s u c c e s s f u l  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 f u n c t i o n   P o l a r D e g r e e s T o C a r t e s i a n (   d M a g ,   d D e g ,   b y r e f   d X ,   b y r e f   d Y   )  
          
         P o l a r D e g r e e s T o C a r t e s i a n   =   f a l s e  
          
         i f   N O T   ( I s E m p t y ( d M a g )   O R   I s E m p t y ( d D e g ) )   t h e n  
                 d X   =   d M a g   *   c o s d ( d D e g )  
                 d Y   =   d M a g   *   s i n d ( d D e g )  
                  
                 P o l a r D e g r e e s T o C a r t e s i a n   =   t r u e  
         e n d   i f  
          
 e n d   f u n c t i o n  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   f u n c t i o n   C a r t e s i a n T o P o l a r D e g r e e s  
 '       C o n v e r t s   p o l a r   c o o r d s   t o   c a r t e s i a n  
 '       R e t u r n s   t r u e   i f   c o n v e r s i o n   i s   s u c c e s s f u l  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 f u n c t i o n   C a r t e s i a n T o P o l a r D e g r e e s (   d X ,   d Y ,   b y r e f   d M a g ,   b y r e f   d D e g   )  
          
         C a r t e s i a n T o P o l a r D e g r e e s   =   f a l s e  
          
         i f   N O T   ( I s E m p t y ( d X )   O R   I s E m p t y ( d Y ) )   t h e n  
                  
                 d M a g   =   s q r (   ( d X   *   d X )   +   ( d Y   *   d Y )   )  
                 d R a d   =   0 . 0  
  
                 i f   (   d M a g   < >   0 . 0   )   t h e n  
                         d R a d   =   A c o s (   d X   /   d M a g   )  
                          
                         i f   (   d Y   <   0 . 0   )   t h e n  
                                 d R a d   =   2 . 0   *   G e t P I ( )   -   d R a d  
                         e n d   i f  
                 e n d   i f  
                  
                 d D e g   =   ( d R a d   *   1 8 0 . 0 )   /   G e t P I ( )  
                                  
                 C a r t e s i a n T o P o l a r D e g r e e s   =   t r u e  
         e n d   i f  
          
 e n d   f u n c t i o n  
  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   f u n c t i o n   C o n v e r t C a r t e s i a n T o P o l a r D e g r e e s  
 '       C o n v e r t s   t o / f r o m   p o l a r   c o o r d s   t o   c a r t e s i a n  
 '       T o P o l a r   -   i n d i c a t e d   i f   w e   a r e   g o i n g   t o / f r o m   p o l a r  
 '       R e t u r n s   t r u e   i f   c o n v e r s i o n   i s   s u c c e s s f u l  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 f u n c t i o n   C o n v e r t C a r t e s i a n T o P o l a r D e g r e e s (   M a g O r X ,   A n g O r Y ,   b y r e f   M a g O r X O u t ,   b y r e f   A n g O r Y O u t ,   T o P o l a r   )  
          
         i f   (   T o P o l a r   )   t h e n  
                 C o n v e r t C a r t e s i a n T o P o l a r D e g r e e s   =   C a r t e s i a n T o P o l a r D e g r e e s (   M a g O r X ,   A n g O r Y ,   M a g O r X O u t ,   A n g O r Y O u t   )  
         e l s e  
                 C o n v e r t C a r t e s i a n T o P o l a r D e g r e e s   =   P o l a r D e g r e e s T o C a r t e s i a n (   M a g O r X ,   A n g O r Y ,   M a g O r X O u t ,   A n g O r Y O u t   )  
         e n d   i f          
          
 e n d   f u n c t i o n  
  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   A d d s   a u t o   d e f i c i e n c y   m e s s a g e   f o r   g i v e n   t a g .    
 '  
 '   F i r s t   c h e c k s   i f   c o r r e s p o n d i n g   a u t o m a t i o n   c a l l   i s   a v a i l a b l e ,   a n d    
 '   i f   n o t   u s e s   o l d e r   A d d D e f i c i e n c y   f u n c t i o n  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S u b   A d d A u t o D e f i c i e n c y ( T a g ,   M e s s a g e )  
          
         '   U s i n g   e r r o r   h a n d l i n g   i n s t e a d   o f   u s i n g   a   v e r s i o n   c h e c k  
         '   f o r   s p e e d ,   s i n c e   G e t V e r s i o n ( )   r e s u l t s   i n   a n   a u t o m a t i o n   c a l l    
         '  
         O n   E r r o r   R e s u m e   N e x t  
         C a l l   A p p l i c a t i o n . A d d A u t o D e f i c i e n c y ( F o r m . F o r m I d ,   T a g ,   M e s s a g e )  
         I f   E r r . N u m b e r   >   0   T h e n  
                 C a l l   A d d D e f i c i e n c y ( M e s s a g e )  
                 E r r . C l e a r  
         E n d   I f  
          
         O n   E r r o r   G o t o   0  
 E n d   S u b  
  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   C l e a r s   a u t o   d e f i c i e n c y   m e s s a g e   f o r   g i v e n   t a g .    
 '  
 '   O n l y   r u n   i f   c o r r e s p o n d i n g   a u t o m a t i o n   c a l l   i s   a v a i l a b l e  
 '  
 ' * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S u b   C l e a r A u t o D e f i c i e n c y ( T a g )  
         O n   E r r o r   R e s u m e   N e x t  
         C a l l   A p p l i c a t i o n . C l e a r A u t o D e f i c i e n c y ( F o r m . F o r m I d ,   T a g )  
         O n   E r r o r   G o t o   0          
 E n d   S u b  
  
 '  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 '   I n i t E n u m e r a t i o n ( )  
  
 '   C r e a t e s   v a r i a b l e s   f o r   a   s e q u e n t i a l   e n u m e r a t i o n    
  
 '   E n u m L i s t   =   " R M _ , a , b , c , d "  
  
 '   c r e a t e s   t h e   f o l l o w i n g  
  
 '   R M _ A   =   0 ,     R M _ B   =   1 ,   R M _ C   =   2   ,   R M _ D   =   3  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 '  
  
 s u b   I n i t E n u m e r a t i o n (   E n u m L i s t   )  
  
         d i m   L i s t ( 1 0 0 )  
  
         S i z e   =   P a r s e S t r i n g (   E n u m L i s t , L i s t , " , " , 1 0 0 )  
  
         P r e f i x   =   L i s t ( 1 )  
  
         f o r   i   =   2   t o   S i z e  
  
                 c a l l   S e t V a r ( P r e f i x   &   L i s t ( i )   ,   i   -   2   )  
  
 '                 c a l l   a l o g ( " I n i t E n u m   "   &   P r e f i x   &   L i s t ( i )   )  
  
         n e x t  
  
 e n d   s u b  
  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t D e g r e e S y m ( )  
 '  
 '   g e t   a   d e g r e e   s y m b o l   f r o m   t h e   t r a n s l a t i o n   t a b l e   o r    
 '   b y   a s c i i   c o d e   t o   a v o i d   m u l t i - b y t e   c h a r a c t e r   p r o b l e m s  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F U N C T I O N   G e t D e g r e e S y m ( )  
     d e g   =   G e t V a r ( " T r a n s D e g r e e S y m " )  
     i f   I s E m p t y ( d e g )   t h e n  
             d e g   =   a p p l i c a t i o n . T r a n s l a t e S t r ( " d e g r e e s y m " )  
             i f   ( S t r C o m p ( d e g ,   " d e g r e e s y m " )   =   0 )   t h e n  
                     d e g   =   " "  
             e n d   i f  
             i f   (   I s B l a n k S t r i n g ( d e g )   )   t h e n  
                 d e g   =   C h r ( 1 7 6 )  
             e n d   i f        
             c a l l   S e t V a r ( " T r a n s D e g r e e S y m " ,   d e g )  
     e n d   i f  
     G e t D e g r e e S y m   =   d e g  
 E N D   F U N C T I O N  
  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S h o w P o p u p C a l e n d a r ( )  
 '  
 '   S h o w   p o p u p   c a l e n d a r .   U s e s   t h e   " R u n T a g C m d   S h o w C a l e n d a r "    
 '   i f   a v a i l a b l e ,   o r   u s e s   t h e   " C a l e n d a r _ P o p u p "   s u b f o r m  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 S u b   S h o w P o p u p C a l e n d a r ( D a t e T e x t F i e l d ,   L o c a t i o n C o n t r o l )  
  
          
         i f   G e t V e r s i o n ( )   > =   1 0 0 0 0   t h e n  
                  
                 c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   L o c a t i o n C o n t r o l ,   " S h o w C a l e n d a r   "   &   D a t e T e x t F i e l d )  
         e l s e  
                 '  
                 '   G e t   t h e   c u r r e n t   v a l u e   o f   t h e   t e x t   f i e l d ,   a n d   s e t   t h a t   v a l u e   a s   t h e   i n i t i a l   v a l u e   f o r    
                 '   t h e   c a l e n d a r   f r o m   t h e   t e x t   i n p u t   f i e l d  
                 '  
                 '  
                 C a l l   S e t V a r (   " c a l p o p u p _ p i c k d a t e " ,   G e t V a r ( D a t e T e x t F i e l d )   )  
  
                 '  
                 '   S p e c i f y   t h e   n a m e   o f   t h e   c o n t r o l   w h i c h   s h o u l d   b e   a s s i g n e d   t h e   s e l e c t e d   d a t e  
                 '  
                 C a l l   S e t V a r (   " c a l p o p u p _ r e t u r n v a r " ,   D a t e T e x t F i e l d   )  
  
                 '  
                 '   D i s p l a y   t h e   p o p u p   f o r m  
                 '  
                 A p p l i c a t i o n . D o M o d a l F o r m (   " C a l e n d a r _ P o p u p "   )  
          
         e n d   i f  
          
  
 E n d   S u b  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t U T C T i m e ( )  
 '  
 '   S h o w   p o p u p   c a l e n d a r .   U s e s   t h e   " R u n T a g C m d   S h o w C a l e n d a r "    
 '   i f   a v a i l a b l e ,   o r   u s e s   t h e   " C a l e n d a r _ P o p u p "   s u b f o r m  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   G e t U T C T i m e ( )  
         L o c a l T i m e   =   n o w ( )  
  
         A c t i v e T i m e B i a s R e g K e y   =   " H K E Y _ L O C A L _ M A C H I N E \ S y s t e m \ C u r r e n t C o n t r o l S e t \ C o n t r o l \ T i m e Z o n e I n f o r m a t i o n \ A c t i v e T i m e B i a s "  
          
         O f f s e t   =   0  
          
         S e t   O S h e l l   =   C r e a t e O b j e c t ( " W S c r i p t . S h e l l " )  
         O n   E r r o r   R e s u m e   N e x t  
         O f f s e t   =   O S h e l l . R e g R e a d ( A c t i v e T i m e B i a s R e g K e y )  
         O n   E r r o r   G o t o   0  
          
         G e t U T C T I m e   =   D a t e A d d ( " n " ,   O f f s e t ,   L o c a l T i m e )  
          
 E n d   F u n c t i o n  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   F o r m a t T i m e F o r S Q L ( )  
 '  
 '   F o r m a t   t i m e   t o   u s e   i n   a n   S Q L   q u e r y .   C h e c k s   t h e   d a t a b a s e   t y p e    
 '   a n d   r e t u r n s   f o r m a t t e d   s t r i n g .  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   F o r m a t T i m e F o r S Q L ( T i m e V a l )  
          
         I f   G e t V a r ( " e A c t i v e D b T y p e " )   =   0   o r   G e t V a r ( " e A c t i v e D b T y p e " )   =   4   T h e n  
                 '  
                 ' S Q L   S e r v e r  
                 '  
                 D a t e E s c   =   " ' "  
         E l s e  
                 '    
                 ' A c c e s s  
                 '  
                 D a t e E s c   =   " # "  
         E n d   I f  
  
         F o r m a t T i m e F o r S Q L   =   D a t e E s c   &   T i m e V a l   &   D a t e E s c  
          
 E n d   F u n c t i o n  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   S t a n d a r d i z e U T C T i m e ( )    
 '  
 '   T a k e s   a   U T C T i m e   a n d   r e m o v e s   a n y   r e g i o n   s p e c i f i c   f o r m a t t i n g   f r o m   i t .    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   S t a n d a r d i z e U T C T i m e ( U t c T i m e )  
          
         h r   =   H o u r ( U t c T i m e )  
         m i n   =   M i n u t e ( U t c T i m e )  
         s e c   =   S e c o n d ( U t c T i m e )  
          
         A M P M S t r   =   " A M "  
         i f   h r   > =   1 2   t h e n  
                 A M P M S t r   =   " P M "  
         e n d   i f  
  
         i f   h r   >   1 2   t h e n  
                 h r   =   h r   -   1 2  
         e n d   i f  
  
 	 S t a n d a r d i z e U T C T i m e   =   F o r m a t D a t e T i m e ( U t c T i m e ,   v b S h o r t D a t e )     &   "   "   &   h r   &   " : "   &   m i n   &   " : "   &   s e c   &   "   "   &   A M P M S t r  
          
 E n d   F u n c t i o n  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t U T C T i m e F o r S Q L ( )    
 '  
 '   G e t   U T C   t i m e   f o r m a t t e d   f o r   u s e   i n   a n   S Q L   q u e r y .    
 '   C h e c k s   t h e   d a t a b a s e   t y p e   a n d   c o r r e c t l y   f o r m a t s   t h e   s t r i n g .  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   G e t U T C T i m e F o r S Q L ( )  
          
         U t c T i m e   =   G e t U T C T i m e ( )  
         U t c S t a n d a r d i z e d T i m e   =   S t a n d a r d i z e U T C T i m e ( U t c T i m e )          
         G e t U T C T i m e F o r S Q L   =   F o r m a t T i m e F o r S Q L ( U t c S t a n d a r d i z e d T i m e )  
          
 E n d   F u n c t i o n  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   B u i l d s   a   c h a r t   l a b e l  
 '  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 f u n c t i o n   B u i l d C h a r t L a b e l ( T a g N a m e ,   L a b e l S t r ,   N d x )  
  
 	 C o l o r S t r   =   G e t C h a r t R G B C o l o r T e x t ( N d x )  
         R G B C o l o r   =   G e t C h a r t R G B C o l o r I n t ( N d x )  
 	 S y m b o l S t r   =   G e t C h a r t S y m b o l T x t ( N d x ,   E m p t y )  
 	 ' L i n e T y p e   =   G e t C h a r t L i n e T y p e ( T e s t N d x ,   E m p t y )  
 	  
 	 F u l l L b l S t r   =   L a b e l S t r   &   " - "   &   C o l o r S t r   &   "   "   &   S y m b o l S t r  
          
         c a l l   S e t N d x V a r ( T a g N a m e ,   N d x ,   F u l l L b l S t r )  
  
         c a l l   A p p l i c a t i o n . R u n T a g C m d ( F o r m . F o r m I d ,   T a g N a m e   &   " _ " &   C S t r ( N d x ) ,   " F g C o l o r       "   +   C S t r ( R G B C o l o r )   )          
      
      
 E n d   F u n c t i o n  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t C h a r t R G B C o l o r T e x t ( )    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   G e t C h a r t R G B C o l o r T e x t ( I n d e x )  
         '   C o l o r   D e f i n i t i o n s  
         C h a r t C o l o r s   =   A r r a y (       " B l u e " ,       " R e d " ,           " G r e e n " ,         " P u r p l e " ,   _  
 	 	 	 	 	 	     " C y a n " ,         " Y e l l o w " ,       " M a g e n t a " ,     " V i o l e t " , _  
 	 	 	 	 	 	     " O r a n g e " ,     " B r o w n " ,         " C o r a l " ,         " M a r o o n " ,   _  
 	 	 	 	 	 	     " O l i v e " ,       " T e a l " ,           " S e a   G r e e n " ,   " L i m e " , _  
 	 	 	 	 	 	     " L t .   G r e e n " ,   " M i d n i g h t   B l u e " ,   " S k y   B l u e " ,             " B e i g e " , _  
 	 	 	 	 	 	     " G r a y " ,     " G r a y " ,       " G r a y " ,       " G r a y " , _  
 	 	 	 	 	 	     " G r a y " ,     " G r a y " ,       " G r a y " ,       " G r a y " , 	 _            
 	 	 	 	 	 	     " G r a y " ,     " L t .   R e d " ,         " B l a c k "   )  
    
         i f   G e t V a r ( " N o Y e l l o w T r a c e s " )     =   1   t h e n    
                 C h a r t C o l o r s ( 5 )   =   " B l a c k "    
         e n d   i f  
          
         i f   G e t V a r ( " N o G r e e n T r a c e s " )     =   1   t h e n    
                 C h a r t C o l o r s ( 2 )   =   " O l i v e "    
         e n d   i f    
    
         G e t C h a r t R G B C o l o r T e x t   =   C h a r t C o l o r s (   (   i n d e x   -   1   )   M o d   3 1 )    
 E n d   F u n c t i o n  
  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t C h a r t S y m b o l T x t ( )    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   G e t C h a r t S y m b o l T x t ( I n d e x ,   D e f a u l t )  
         D i m   S e l e c t e d S y m b o l  
         S y m b o l s   =   A r r a y ( " C i r c l e " , " T r i a n g l e " , " T r i a n g l e " , " S q u a r e " , " D i a m o n d " )  
  
         I f   (   i n d e x   >   5   )   T h e n  
 	 I f   I s E m p t y   (   D e f a u l t   )     T h e n  
 	         S e l e c t e d S y m b o l   =   S y m b o l s (   (   i n d e x   -   1   )   M o d   5 )  
 	 E l s e  
 	         S e l e c t e d S y m b o l   =   D e f a u l t  
 	 E n d   I f  
         E l s e  
 	 S e l e c t e d S y m b o l   =   S y m b o l s (   i n d e x   -   1   )  
         E n d   I f  
  
         G e t C h a r t S y m b o l T x t   =   S e l e c t e d S y m b o l  
 E n d   F u n c t i o n  
  
  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   G e t C h a r t R G B C o l o r I n t ( )    
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 F u n c t i o n   G e t C h a r t R G B C o l o r I n t ( I n d e x )  
         '   C o l o r   D e f i n i t i o n s  
         C h a r t C o l o r s   =   A r r a y (       C L R _ B L U E ,       C L R _ R E D ,           C L R _ G R E E N ,       C L R _ P U R P L E ,   _  
 	 	 	 	 	 	   C L R _ C Y A N ,         C L R _ Y E L L O W ,       C L R _ M A G E N T A ,     C L R _ V I O L E T , _  
 	 	 	 	 	 	     C L R _ O R A N G E ,     C L R _ B R O W N ,       C L R _ C O R A L ,         C L R _ M A R O O N ,   _  
 	 	 	 	 	 	   C L R _ O L I V E ,       C L R _ T E A L ,           C L R _ S E A G R E E N ,   C L R _ L I M E , _  
 	 	 	 	 	 	     C L R _ L T G R E E N ,   C L R _ M I D N I G H T ,   C L R _ S K Y ,           C L R _ B E I G E , _  
 	 	 	 	 	 	     C L R _ G R A Y 1 0 ,     C L R _ G R A Y 2 0 ,       C L R _ G R A Y 3 0 ,       C L R _ G R A Y 4 0 , _  
 	 	 	 	 	 	     C L R _ G R A Y 5 0 ,     C L R _ G R A Y 6 0 ,     C L R _ G R A Y 7 0 ,       C L R _ G R A Y 8 0 , 	 _            
 	 	 	 	 	 	     C L R _ G R A Y 9 0 ,     C L R _ L T R E D ,       C L R _ B L A C K   )  
          
         i f   G e t V a r ( " N o Y e l l o w T r a c e s " )   =   1   t h e n    
                 C h a r t C o l o r s ( 5 )   =   C L R _ B L A C K  
         e n d   i f  
          
         i f   G e t V a r ( " N o G r e e n T r a c e s " )     =   1   t h e n    
                 C h a r t C o l o r s ( 2 )   =   C L R _ O L I V E  
         e n d   i f    
    
         G e t C h a r t R G B C o l o r I n t   =   C h a r t C o l o r s (   (   i n d e x   -   1   )   M o d   3 1 )    
 E n d   F u n c t i o n  
  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 '   U p d a t e A s L e f t T a g s ( P r e f i x ,   T a g L i s t )    
 '  
 '   U p d a t e s   t h e   A s   L e f t   t a g s   b y   t a k i n g   a   l i s t   o f   A s   F o u n d   t a g s  
 '   a n d   t h e   p r e f i x   o f   t h e   A s   L e f t   s u b f o r m  
 '  
 '   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
  
 f u n c t i o n   U p d a t e A s L e f t T a g s ( P r e f i x ,   T a g L i s t )  
          
         A F T a g s   =   S p l i t ( T a g L i s t ,   " , " )  
          
         f o r   e a c h   C u r r e n t T a g   i n   A F T a g s  
                 i f   C u r r e n t T a g   < >   e m p t y   t h e n  
                 A F T a g N a m e   =   S p l i t ( C u r r e n t T a g ,   " _ _ " )          
                         i f   A f T a g N a m e ( 1 )   < >   E m p t y   t h e n  
                                 A L T a g N a m e   =   P r e f i x   &   " _ _ "   &   A f T a g N a m e ( 1 )  
                                 c a l l   A p p l i c a t i o n . U p d a t e A s F o u n d A s L e f t ( F o r m . f o r m I d ,   C u r r e n t T a g ,   A L T a g N a m e )  
                         e n d   i f  
                 e n d   i f  
         n e x t  
          
 e n d   f u n c t i o n  
  
     mE#џўџD a t a џўџџ\
I n t e r n a t i o n a l N o t e s :  
  
 J L T   8 / 2 8 / 2 0 0 9  
 p e r   b r u c e :   S e t V i s i b l e   -   h a n d l e   m u l t i l a n g u a g e   c a s t s   o f   t r u e / f a l s e  
  
 J L T   8 / 2 1 / 2 0 0 9  
 p e r   b r u c e :   G e t R o w F r o m T a g   i n i t   t o   0   /   r e m o v e d   a l o g   f r o m   P o s t O n U p d a t e 2  
  
 D J S   8 / 1 7 / 2 0 0 9  
 A d d   A s s e t I d _ S N   t o   s a v e d   d a t a  
  
 R T A   6 / 1 7 / 2 0 0 9  
 A d d e d   S u b   M a p T a g s ( )  
  
 R T A   4 / 1 7 / 2 0 0 9  
 A d d e d   t h e   f u n c t i o n s   S e t T e m p e r a t u r e C ( ) ,   S e t T e m p e r a t u r e F ( ) ,   G e t T e m p e r a t u r e C ( ) ,   G e t T e m p e r a t u r e F ( )  
 A d d e d   D e c l a r e V a r ( " T e m p e r a t u r e C " )   t o   O n I n i t i a l U p d a t e  
  
 J L T   9 / 9 / 0 8  
 f i x e d   g e t V e r s i o n ( )   c h e c k   f o r   " t r u n k "   ( r e t v a l   i s   1   b a s e d ;   0   m e a n   n o t   f o u n d )  
  
 B A B   9 / 9 / 0 8  
 U p d a t e   L o a d C u r v e   m e t h o d   f o r   o c r  
  
 J L T   0 3 / 0 6 / 0 8  
 f i x e d   G e t V e r s i o n ( )   t o   s u p p o r t   c r e a t i o n   o f   d o u b l e   r e g a r d l e s s   o f   d e c i m a l   s e t t i n g   ( c o m m a   o r   p e r i o d )  
  
 B A B   9 / 4 / 2 0 0 8  
 A d d S a v e d V a r ( )   m e t h o d   f o r   m e t r i c s    
  
 R T A   2 / 1 6 / 2 0 0 6  
  
 A d d e d   f u n c t i o n s   g e t V e r s i o n   a n d   T r a n s l a t e .  
 g e t V e r s i o n   c a l l s   a p p l i c a t i o n . g e t V e r s i o n ,   p a r s e s   t h e   r e s u l t   a n d   s e t s   a   v a r i a b l e   n a m e d   " P d b V e r s i o n " .  
 T r a n s l a t e   c h e c k s   w h e t h e r   t h e   a p p l i c a t i o n . t r a n s l a t e S t r   m e t h o d   e x i s t s   b e f o r e   c a l l i n g   i t .   I f   i t  
 d o e s n ' t   e x i s t ,   n o   t r a n s l a t i o n   i s   d o n e .  
 : E N D  
  
 '  
 '   * * * *     D o   n o t   c h a n g e   t h e s e   i t e m s     * * * *  
 '  
 '   I f   y o u   n e e d   t h e m   t o   b e   d i f f e r e n t   p l e a s e   c o p y   a n d   p a s t e   t h e m   i n t o    
 '   t h e   U s e r D a t a   t e m p l a t e   a n d   c h a n g e   t h e m   t h e r e  
 '  
 '   e x a m p l e   o f   s c r i p t   t o   u s e   l i s t s   i n   d r o p d o w n      
 ' . t h i s ! l i s t   =   a p p l i c a t i o n . R e a d D a t a M u l t i L i n e F r o m T a g ( . F o r m I d ,   " C a b l e I n s u l a t i o n T y p e s : " )  
 '    
  
 T e s t L e a d C o n f i g :  
 U S T :   M e a s u r e   R e d ,   G r o u n d   B l u e  
 U S T :   G r o u n d   R e d ,   M e a s u r e   B l u e  
 U S T :   M e a s u r e   R e d   a n d   B l u e  
 G S T :   G r o u n d   R e d   a n d   B l u e  
 G S T :   G u a r d   R e d ,   G r o u n d   B l u e  
 G S T :   G r o u n d   R e d ,   G u a r d   B l u e  
 G S T :   G u a r d   R e d   a n d   B l u e  
 : E N D  
  
 M a n _ I n f o :  
 G E  
 A B B  
 W e s t i n g h o u s e  
 : E N D  
  
  
 C a b l e I n s u l a t i o n T y p e s :  
 T H H N  
 T W  
 T H W  
 X L P  
 X P  
 X L  
 X L P E  
 H M W - P E  
 E D  
 E P R  
 E T H E L E Y N E  
 P E  
 E P P M  
 S F - 2  
 F R E P  
 E T E E  
 P V C  
 K Y N A R  
 T E F L O N  
 T E F Z E L  
 K A P T O N  
 E P D M  
 T R X P  
 : E N D  
  
  
 B u s h T y p e C l a s L i s t :  
 O + C  
 G O   ( 2 5 - 7 6 5   k V )  
 C T F   ( 2 0 - 6 0   k V )  
 C T F   ( 8 5 - 3 3 0   k V )  
 C T K F   ( 2 0 - 6 0   k V )  
 C T K F   ( 8 5 - 3 3 0   k V )  
 B  
 F  
 L  
 L C  
 L I  
 L M  
 O F  
 O F I  
 O F M  
 S  
 S I  
 S I M  
 T   ( G E )  
 T   ( A B B )  
 U  
 C O T  
 C O T A  
 C O S  
 S O T  
 E R C   ( 1 5 - 2 3   k V )  
 P R C   ( 1 5 - 6 9   k V )  
 P O C   ( 1 5 - 7 6 5   k V )  
 P  
 P A  
 P B  
 W T x F  
 M I C . ( 2 5 - 6 9   k V )  
 M I C . ( A b o v e   6 9   k V )  
 G   ( 4 6 - 1 3 8   k V )  
 L   ( 4 6 - 1 3 8   k V )  
 L   ( 7 . 5 - 3 4 . 5   k V )  
 G K   ( 1 5 - 5 0 0   k V )  
 L K   ( 1 5 - 6 9   k V )  
 P T A O   ( 2 5 - 2 3 0   k V )  
 3 P V . F   ( 2 5 - 2 3 0   k V )  
 C O N D .   E X C E P T   O  
 O  
 O - A 1  
 O C  
 O +  
 A  
 L C R J  
 A B  
 G O B  
 G O H  
 G O A  
 G O E  
 G O M  
 G S A  
 R T K F  
 R T X F  
 G S B  
 O T H E R  
 : E N D  
  
 V o l u m e L i s t :  
 U G  
 I G  
 L  
 k g  
 L B  
 T N  
 : E N D  
  
 C o n d i t i o n L i s t :  
 A c c e p t a b l e  
 D i r t y  
 E x c e l l e n t  
 G o o d  
 N / A  
 N / T  
 N e w  
 P o o r  
 V e r y   P o o r  
 : E N D  
  
 C l e a n L i s t :  
 C  
 C / L  
 L  
 N / A  
 : E N D  
  
 '  
 '   * * * *     D E F A U L T   C O M M E N T S     * * * *  
 '  
  
 D e f a u l t C o m m e n t s :  
  
 T e s t   O K  
  
 : E N D  
  
  
 '  
 '   * * * *     D E F A U L T   D E F I C I E N C I E S     * * * *  
 '  
  
 D e f a u l t D e f i c i e n c i e s :  
  
 T e s t   F a i l e d  
  
 : E N D     џўџ  џџ  OPdbFormsEmbeddedWsts5;iE#      ?  ?                           џўџC u s t o m   F i e l d   E n t r y џўџ tE#џџџ                                 iF#	             џџ   OOptDrawBorderData                                џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџџџ   OFormsToolEmbeddedWs76       5>                lE#  џўџ      ЭЭЭЭ   џўџD a t e M o d i f i e d        ЋiЅф@џўџD a t e C r e a t e d        ЃяЎт@џўџ	D a t e S y n c h        ЃяЎт@џўџb I s D e l         џўџ
D e l e t e I n f o    џўџb r u c e b - > џўџ
b I s R e t i r e d        џўџb O p t i m a F o r m        џўџb S u g g e s t U p d a t e        џўџ	L a s t M o d B y    џўџ џўџC r e a t o r G u i d    џўџ џўџT e m p l a t e s U s e d    џўџ%U s e r   D a t a | O p t i m a   D a t a   -   D o   N o t   C h a n g e џўџE x t e r n a l T e m p l a t e s    џўџ џўџT e m p l a t e F a m i l y    џўџ џўџT e m p l a t e M a n u    џўџ џўџb I s L o c a l F o r m         џўџS y n c h W r i t e D a t e        Gь;ЖТ+п@џўџS t a t F l g         џўџb L o c k e d        џўџL o c k I n f o    џўџ џўџoA Q Q B A A A O V O 7 K N B I R W X C B 
 T e m p l a t e :   U s e r   D a t a 
 D a t e   C r e a t e d :   ! 2 0 0 4 , 0 9 , 3 0 , 0 9 , 4 2 , 2 5 
 D a t e   M o d i f i e d :   ! 2 0 1 3 , 0 4 , 0 7 , 0 0 , 4 5 , 3 7    џўџT e m p l a t e G U I D    џўџA Q Q B A A A O V O 7 K N B I R W X C B џўџN a m e    џўџ	U s e r   D a t a џўџ
D e f i n i t i o n    в optform          SAVDOUT{s65>?   >  >                =                         џџ  OOptDrawCtrlScriptmE#On Initial Update     mE#	On Updateџэ
if GetVar("PdbDeviceGuid")  = "zk00037r@\]d2/Џ2" then 
    if GetVar("Manufacturer_List") <> Empty and GetVar("Mfr") = Empty then 
        call SetVar("Mfr", GetVar("Manufacturer_List") )
        call SetVar("Manufacturer_List",Empty)
        call SetVar("SerNo", GetVar("SerialNo") )
        call SetVar("VA_1", GetVar("kva1") )
        call SetVar("VA_2", GetVar("kva2") )
        call SetVar("COOLANT", GetVar("COOLANT") )
        call SetVar("Phases", GetVar("Phase") )
        call SetVar("TankType", GetVar("TYPE") )
        call SetVar("Type", GetVar("CLASS") )
        call SetVar("WdgMaterial", GetVar("windmaterial") )
        call SetVar("Weight",Empty)
        call SetVar("Bil", CStr(GetVar("KVPri") ) )
        call SetVar("Weight", GetVar("TotalWeight") )
        call SetVar("Gallons", GetVar("maintankcap") )

        '
        ' ERS to provide mapping
        '
        if .TYPE = "Sealed Gas Filled Dry Type" then .TankType = "Sealed"




    end if 
end if      mE#On Page Set Focus     mE#On Form Exit     mE#On Timer 	    mE#Global Functions && VariablesRfunction MakeSQLSafe(text)
    MakeSQLSafe=Replace(text,"'","''")
end Function
    mE#DataџџџNb BATTERIES_Comments: 

'DC BATTERIES/CHARGERS

'Battery Bank - General
Battery banks are the last barrier between critical equipment operation and catastrophe. Good maintenance practices include load testing to \\\
ensure the batteries can provide the required current if the need arises. A thermographic inspection during load tests can reveal dangerous \\\
conditions. Connections not providing sufficiently low contact resistance resulting from corrosion and improper torque are easily identified \\\
under loaded conditions using thermography. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\
:END

BATTERIES_Deficiencies: 

'DC BATTERIES/CHARGERS

SEDIMENT/CORROSION ----- Batteries show signs of sediment and/or corrosion. Recommend performing preventive maintenance and testing to determine the condition of the \\\
battery system. Recommend cleaning all connections, testing for connection resistance, electrolyte specific gravity, cell resistance, charger \\\
float and equalize settings, and perform a load test to determine battery system capacity and overall condition. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

CORROSION ----- The batteries show signs of corrosion. Recommend performing cleaning and connection resistance testing of the battery bank connections. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

SPECIFIC GRAVITY ----- Batteries tested have specific gravity readings of the electrolyte that deviate significantly from expected values. Recommend replacing these \\\
batteries. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

CABLES/BUS DUCT_Comments: 

'CABLES

'Cable Test Period
Cables should be inspected annually and tested every 3 to 5 years. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Cables- New OK
High potential tests were used to determine the electrical integrity of the cable systems tested. Cable systems tested included the terminations. \\\
The main objective of this test is to make sure the dielectric of the cables has not been damaged and to determine the integrity of the stress \\\
relief devices. Cables tested passed the high potential test. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Cables-Existing OK
High potential tests were used to determine the electrical integrity of the cable systems tested. Cable systems tested included the terminations. \\\ 
The main objective of this test is to make sure the dielectric of the cables has not been damaged and to determine the integrity of the stress \\\
relief devices. Cables tested passed the high potential test. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'BUSWAY

:END

CABLES/BUS DUCT_Deficiencies: 

'BUSWAY

'CABLES

:END

CALIBRATION_Comments: 

:END

CALIBRATION_Deficiencies: 

:END

CIRCUIT BREAKER_Comments: 

'CIRCUIT BREAKERS

'LOW VOLTAGE

'CB-Static OK
High Current, contact resistance, and insulation resistance tests were used to determine the electrical integrity of the circuit breakers tested. \\\
High current testing, which is the only reliable method of checking static overcurrent trip devices, was used to verify the complete tripping \\\
system. Circuit breakers tested passed the high current, contact resistance and insulation resistance tests. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'MEDIUM/HIGH VOLTAGE

'MV CB Test Period
Medium Voltage Circuit Breakers should be inspected, cleaned, lubricated (as necessary), tested, and function checked every 1 to 3 years based on \\\
operating environment and application. MV circuit breakers should have the following tests performed: insulation resistance, insulation power \\\
factor, and contact resistance. Breakers require exercising to assure proper operation and distribution of lubricants. Breakers should be \\\
exercised annually (minimum recommendation). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'MVCB-Tests OK
Power factor, contact resistance and insulation resistance tests were used to determine the electrical integrity of the medium voltage air \\\
circuit breakers tested. Test results were compared to data in our files of similar breakers. Medium voltage breakers tested passed the \\\
insulation power factor, contact resistance and insulation resistance tests. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'OCB-Tests OK
Power factor/Tank-Loss Index, contact resistance and insulation resistance tests were used to determine the electrical integrity of the medium \\\
voltage oil circuit breakers tested. Test results were compared to data in our files of similar breakers. Medium voltage breakers tested passed \\\
the insulation power factor, contact resistance and insulation resistance tests. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

CIRCUIT BREAKER_Deficiencies: 

'CIRCUIT BREAKERS

LOW INSULATION RESISTANCE ----- The insulation resistance test values were less than recommended by NETA MTS standards.  Recommend additional investigation to determine \\\
cause of deteriorated insulation resistance.  Replacement of breaker components such as pole pieces may be necessary to improve insulation resistance. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


'LOW VOLTAGE

'CB-Replace
This circuit breaker did not trip within acceptable time limits when tested by primary current injection. Recommend replacing this breaker. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'CB-Replace Trip Unit
This circuit breaker did not trip within acceptable time limits when tested by primary current injection. Recommend replacing the trip unit \\\
and re-testing following the repairs. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

INJECTION TIMING TEST ----- The long time element on one or more poles of this breaker does not conform to the manufacturer''s published time curves.  The principal \\\
purpose of field testing is not to determine if the breaker exactly meets published time curves, but rather to determine if the circuit breaker is furnishing the \\\
protection for which it was installed.  A circuit breaker that trips in less than the minimum time shown by the trip time curves will furnish protection at a lower \\\
current level.  Unless this results in nuisance tripping or coordination problems, this condition is may be considered acceptable. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

INJECTION TIMING TEST ----- Instantaneous element on one or more poles of this breaker does not conform to the manufacturer''s published time curves.  The principal purpose \\\
of field testing is not to determine if the breaker exactly meets published time curves, but rather to determine if the circuit breaker is furnishing the protection for which \\\
it was installed.  A circuit breaker that trips on less current than the minimum current shown on the manufacturer''s published time curves will furnish protection at a lower \\\
current level.  Unless this results in nuisance tripping or coordination problems with the circuit, this condition may be considered acceptable. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

INJECTION TIMING TEST ----- Circuit breaker trips outside the manufacturer''s recommended trip curve. Investigate and/or repair/replace trip unit. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

INJECTION TIMING TEST ----- Circuit breaker failed to trip.  Replace breaker/trip unit. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

REPLACE DEFECTIVE/ANTIQUATED TRIP UNIT ----- This circuit breaker did not trip within acceptable time limits when tested by primary current injection.  The trip unit \\\
in this type of breaker is an electro-mechanical series overcurrent device.  These devices have sucker discs that become sticky, mechanisms \\\
that corrode, small air or oil dashpot orifices that become clogged, rubber diaphragms that rupture, etc.  We recommend retrofitting this breaker \\\
with a static trip device with electronic current sensing and trip timing.  The static trip units are reliable devices, with precise, repeatable tripping characteristics. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

REPLACE DEFECTIVE TRIP UNIT ----- This circuit breaker did not trip within acceptable time limits when tested by primary current injection.  Recommend replacing the \\\
trip unit and re-testing following the repairs. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


'MEDIUM/HIGH VOLTAGE

'MVCB-Moisture/Investigate
Power factor insulation tests on this medium voltage air circuit breaker indicates deteriorated insulation. The bushing power factor test at \\\
2,500 volts (breaker open, opposite bushing guarded) indicated power factors greater than six percent. Data in our files suggests that a power \\\
factor of less than 6% may be more normal. Our test results indicate moisture has been absorbed by the circuit breaker insulation. \\\
The following is recommended:  (1) All openings to the breaker cubicle should be eliminated. Particular attention should be given to  openings \\\
from conduits, cable ducts or trenches. (2) Most cubicles require some venting, however, certain old breaker designs include top and bottom \\\
openings, where a limited venting would be more suitable.  (3) Heaters should be mounted at the cubicle floor level on either side of the breaker. \\\	
Approximately 1500 watts of heat is required to reduce moisture in the breaker.  Circuit breaker should be removed from its cubicle in dried. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'CB-IR/Deteriorated
The insulation resistance test results are rated deteriorated. Additional investigation is necessary to determine cause of deteriorated \\\
insulation resistance. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'MVCB-Moisture/Investigate
Insulation power factor insulation tests on this medium voltage air circuit breaker indicates deteriorated insulation. The bushing power factor test \\\
results (breaker open, opposite bushing guarded) indicates power factors greater than 6%. Power factor test values of less than 6% may be more \\\
normal. High power factor test results are commonly caused from moisture absoption of the equipment insulation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'MVCB-Moisture/Investigate
Insulation power factor tests on this medium voltage air circuit breaker indicates deteriorated insulation. The bushing power factor test results \\\
(breaker open, opposite bushing guarded) are elevated.  Test results indicate moisture has been absorbed by the circuit breaker insulation. \\\
Recommend the following actions:  (1) All openings to the breaker cubicle should be eliminated. Particular attention should be given to openings \\\
from conduits, cable ducts or trenches. Most cubicles require some venting, however, certain old breaker designs include top and bottom openings, \\\
where a limited venting would be more suitable.  (2) Heaters should be mounted at the cubicle floor level on either side of the breaker. \\\
(3) Approximately 1500 watts of heat is required to reduce moisture in the breaker. (4) Circuit breaker should be removed from its cubicle \\\ 
in dried. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

The following actions are recommended: (1) All openings to the breaker cubicle should be eliminated. Particular attention should be given to openings\\\
from conduits, cable ducts or trenches. Most cubicles require some venting, however, certain old breaker designs include top and bottom openings, where \\\
a limited venting would be more suitable.  (2) Heaters should be mounted at the cubicle floor level on either side of the breaker.  (3) Approximately \\\
1500 watts of heat is required to reduce moisture in the breaker. (4) Circuit breaker should be removed from its cubicle and dried. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

COORDINATION DATA_Comments: 

:END

COORDINATION DATA_Deficiencies: 

:END

DISCONNECTS/SWITCHES_Comments: 

'SWITCHES

'LOW VOLTAGE SWITCHES
Contact resistance measurements and insulation resistance tests were performed on the low voltage switches.  Insulation resistance tests \\\
are used to determine the quality or condition of the switch insulation.  Contact resistance measurements are used to determine the quality of \\\
the switch contacts.  The contact resistance and insulation resistance test results for this equipment was found to meet applicable NETA standards. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

DISCONNECTS/SWITCHES_Deficiencies: 

'SWITCHES


MECHANICAL OPERATION ----- Switch does not operate reliably.  Recommend exercising and lubricating switch. Additional maintenance may be required. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

EQUIPMENT FIELD SURVEY_Comments: 

:END 

EQUIPMENT FIELD SURVEY_Deficiencies: 

:END 

GENERATORS_Comments: 

:END

GENERATORS_Deficiencies: 

:END

GROUND FAULT_Comments: 

'GROUND FAULT PROTECTION

'Ground Fault-Purpose
The purpose of testing a ground fault system is to ensure continued correct operation of the ground fault protective equipment. Deterioration of \\\
electrical equipment is a normal process. Since the deterioration of electrical systems can lead to equipment malfunction and electrical failure, \\\
the National Fire Protection Association in NFPA 70B recommends periodic testing of ground fault systems to confirm operation of the relays. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Ground Fault-Test OK
Ground fault relays were tested by injecting current into the relays and measuring for pickup and tripping time values. All relays tested were \\\
found operational requiring minor cleaning and no calibration adjustments. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Ground Test-ok
Ground fault relays were tested by injecting current into the relays and measuring for pickup and tripping time values. All relays tested were \\\
found operational requiring minor cleaning and no calibration adjustments. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

GROUND FAULT_Deficiencies: 

'GROUND FAULT PROTECTION

:END

GROUND MAT (EARTH) GROUNDING TESTS_Comments: 

'GROUNDING SYSTEMS

'Ground Test-Purpose
The purpose of ground resistance tests is to determine the effectiveness of the grounding electrode system which is used to protect \\\
personnel and equipment. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Adequate distance from the ground system under test was not available because of geographic limitations. Alternative ground testing was performed. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

GROUND MAT (EARTH) GROUNDING TESTS_Deficiencies: 

'GROUNDING SYSTEMS

HIGH RESISTANCE ----- Ground resistance measurement exceeds specification. Ground system requires modification to meet specification. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

HIGH RESISTANCE ----- Point-to-point resistance value exceeds 0.5 ohms. Investigate cause of high test result. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

The soil conditions should be reviewed to determine the possible causes for the unacceptable results and that corrective actions be implemented. \\\
A retest of the system should be completed after modifications to ensure the desired ground resistance is achieved. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

The ground resistance was found to be unacceptable in accordance with the project specification requirement of 5 ohms.  Supplemental grounding \\\
electrodes or other means will be required to meet the specifications requirements.  This can be accomplished by many alternatives including extending \\\
electrode deeper, adding electrodes and using alternate types to increase the contact surface area with the earth.  The choice of methods depends on the \\\
coil soil conditions that are causing an unsatisfactory test result. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

The neutral feeder conductors are grounded downstream of the neutral disconnect link.  Neutral grounds will desensitize the ground fault system \\\
and may cause nuisance tripping. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Ground Test-Recommendations
The soil conditions should be reviewed to determine the possible causes for the unacceptable results and that corrective actions be implemented. \\\
A retest of the system should be completed after modifications to ensure the desired ground resistance is achieved. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

INFRARED_Comments: 

'INFRARED/THERMOGRAPHIC INSPECTION

Following all repairs, a follow-up thermographic inspection should be scheduled to assure the deficiencies have been adequately repaired. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

INFRARED_Deficiencies:

'INFRARED/THERMOGRAPHIC INSPECTION

ELEVATED TEMPERATURE ----- Contacts of this device exhibit elevated temperature.  Recommend additional investigation and repairs. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

ABNORMAL HEATING ----- Abnormal heating of this equipment was detected.  Recommend additional investigation and make necessary repairs. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

ELEVATED CONNECTION TEMPERATURE ----- Connection(s) of this equipment has/have elevated temperatures.  Recommend cleaning and checking connections tight. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Many of the deficiencies noted involve fused disconnect switches.  These deficiencies usually result from one or more of the following: \\\
(1) Loose or corroded connections.  These connections should be disassembled, cleaned, and tightened.  Be aware that there are sometimes \\\
hidden connections behind the switch that require disassembly and cleaning,  (2) Deteriorated switch mechanisms.  Pivot points and blade \\\
connections sometimes wear and create a high resistance path.  Disassembly, cleaning, or replacement is sometimes necessary,  (3) Fuse clip \\\
deterioration.  Over time, fuse clips can be come loose and create a high resistance path.  Fuse clips that are worn and no longer provide \\\
the proper retaining tension should be replaced. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Other common deficiencies involve motor starter overload devices.  Recommendations for overload overheating are: (1) Verify overloads are a \\\
matched set and properly sized,  (2) verify motor currents are balanced,  (3) Verify connections are clean and tight. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

INSTRUMENT TRANSFORMERS_Comments: 

'INSTRUMENT TRANSFORMERS

:END

INSTRUMENT TRANSFORMERS_Deficiencies: 

'INSTRUMENT TRANSFORMERS

:END

INSULATION FLUID_Comments: 

'INSULATING FLUID

:END

INSULATION FLUID_Deficiencies: 

'INSULATING FLUID

Fluid seepage/leaks were noted at the equipment bushings. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Fluid seepage/leaks were noted at the equipment covers. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Fluid seepage/leaks were noted at the drain valve. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Fluid seepage/leaks were noted at the sample valve. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Fluid seepage/leaks were noted at the radiators. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Low Liquid
LIQUID LOW ----- Based on the liquid level gauge, the liquid level of this equipment is below its normal level. Recommend adding the necessary liquid to \\\
the proper level. A low liquid level condition can result in excessive heating and insulation damage. Adequate liquid is necessary to permit \\\
proper circulation and cooling. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Liquid Level
The fluid level cannot be determined by the level gauge. Recommend investigating fluid level and replacing gauge to determine fluid level \\\
during future inspections. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Fluid-Slightly Low
Based on the liquid level gauge, the liquid level of this equipment is slightly low. During a future maintenance project, recommend adding the \\\
necessary liquid to the proper level. A low liquid level condition can result in excessive heating and insulation damage. Adequate liquid is \\\
necessary to permit proper circulation and cooling. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

A small leak at the sample valve was noted. Sample valve should be monitored to determine if the leak continues. Re-packing of the existing \\\
valve or replacement may be necessary \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Recommend repairing the leak noted at the sample valve. Re-packing of the existing valve or replacement is necessary \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

FLUID DETERIORATED ----- Insulation fluid is rated deteriorated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

HIGH GASSES ----- Insulating fluid has high dissolved gas content. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

HIGH MOISTURE ----- Insulating fluid has high moisture content. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

FREE WATER ----- Insulating fluid has free water. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

METERS & TRANSDUCERS_Comments: 

'METERS
Verification of the switchboard ammeter and voltmeter accuracy has been accomplished by comparison to test sets maintained by High Voltage \\\
Maintenance Corporation. The accuracy of these test sets are traceable to the National Standards maintained by the National Institute of Standards \\\
in Washington, D.C. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

METERS & TRANSDUCERS_Deficiencies: 

'METERS
Meter is out of calibration. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Meter cover is broken and requires replacement. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Meter is damaged and requires replacement. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

MISCELLANEOUS_Comments: 

'CAPACITORS

:END

MISCELLANEOUS_Deficiencies: 

'CAPACITORS

:END

MOTOR CONTROL CENTERS_Comments: 

'MOTOR CONTROL CENTERS

'MOTOR CONTROL STARTERS

:END

MOTOR CONTROL CENTERS_Deficiencies: 

'MOTOR CONTROL CENTERS

'MOTOR CONTROL STARTERS

:END

POWER FACTOR TESTS_Comments: 

'BUSHINGS

'Bushings-High MW Losses
Hot-collar tests on bushings are highly susceptible to surface contamination and it can be difficult to rid bushings of all surface contamination \\\ 
in the field. Use of cleaners and heat guns can be used to improve the bushing condition for testing purposes. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'SURGE ARRESTERS

'Arresters-ok
Power factor and insulation resistance tests were used to determine the electrical integrity of the lightning arresters tested. The main \\\
objective of this test is to detect contamination by moisture and/or dirt, corroded gap, cracked porcelain, broken shunting resistors, broken \\\
elements, or improper assembly which would cause abnormal losses. Arresters tested passed the insulation power factor and insulation resistance \\\
tests. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

POWER FACTOR TESTS_Deficiencies: 

'BUSHINGS

BUSHING LEAK ----- A fluid leak from one or more bushings was noted during this project. Recommend scheduling an outage to make necessary repairs \\\
to eliminate the leak. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHING HIGH LOSSES ----- Bushing tests performed on this equipment indicates higher than normal milliwatts loss on the one or more bushing. \\\
Compiled test data suggests the milliwatts losses measured on this bushing exceeded the values normally expected. Also of concern is a \\\
comparison of this bushing to other bushings on this and other similar equipment. Abnormally high milliwatts test values most likely reflect \\\
contamination within the bushing which has entered through cracks in the porcelain or a defective seal. Recommend replacing this bushing at \\\
your next convenient opportunity. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHINGS HIGH LOSSES ----- A significant deviation in milliwatts-losses was noted between similar bushings. Normal acceptable deviations between similar bushings is \\\
1.25 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHING HIGH LOSSES ----- A comparison of the watts-loss test results indicates a higher than normal test result on one ore more bushings compared to other similar \\\
bushings and/or industry data on similar bushings.  The following conditions should be considered in the overall evaluation of these bushings: \\\ 
(1) Hot-collar tests on bushings are highly susceptible to surface contamination and it can be difficult to rid bushings of all surface \\\
contamination in the field. Use of cleaners and heat guns can be used to improve the bushing condition for testing purposes,  (2) A deviation \\\
in milliwatts-losses greater than 200% (2.3 mW) exists between bushings. Normal acceptable deviations between similar bushings is 1.25 mW \\\
(Doble Engineering Test-Data Reference Book),  (3) Serious concern is given for bushings with hot-collar test results greater than 6 mW \\\
(Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHING HIGH POWER FACTOR ----- Power factor test conducted on the bushings of this equipment indicates a problem in one or more bushings. A power factor \\\
test is probably the most reliable indication of moisture, contamination or cracks in a bushing. Power factor of sealed bushings is low \\\
initially and remains low in service if the bushing is in good condition. An increase in power factor or a poor reading is evidence of a change \\\
in the characteristics and evidence that a potentially damaging condition is developing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHINGS DIRTY ----- The equipment bushings are dirty and require cleaning. Bushing surface contamination often results in noisy substations, \\\
damage to insulating surfaces, partial discharge, tracking flashover and eventually loss of power. It is possible to prevent bushings from \\\
flashing over by periodic cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Serious concern is given for bushings with hot-collar test results greater than 6 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Insulation power factor tests conducted on the bushings indicates possible problems with one or more bushings. An insulation power factor test \\\
is probably the most reliable indication of moisture, contamination or cracks in a bushing. An increase in power factor or a poor reading is \\\
evidence of a change in the characteristics and evidence that a potentially damaging condition is developing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

A significant deviation in milliwatts-losses was noted between similar bushings. Normal acceptable deviations between similar bushings is \\\
1.25 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Serious concern is given for bushings with hot-collar test results greater than 6 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Additional recommendations for the deteriorated bushing(s) can be provided after the next set of tests are performed and results are evaluated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

While it is impossible to guarantee the continued reliable operation of any device based on test data alone, the test results of this bushing \\\
cause concern for its continued reliable operation. The test results do not necessarily indicate the bushing will fail in the immediate future \\\
but they do deviate from the norm enough to cause concern and require additional follow up investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Without regard to costs, the most conservative action would be to replace this bushing. However, based on the test results and the above \\\
evaluation criteria, the more reasonable response would be to monitor the bushing and perform additional tests within the next 12 months. \\\
These tests are performed to validate the test results recently obtained and also permit additional investigative tests to be performed \\\
to better identify the potential defect of this bushing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'SURGE ARRESTERS
WATTS LOSS DETERIORATED ----- Insulation power factor (watts loss) and insulation resistance tests were used to determine the electrical integrity of the \\\ 
arresters tested. The main objective of these tests is to detect contamination by moisture and/or dirt, corroded gap, cracked porcelain, broken shunting \\\
resistors, broken elements, or improper assembly which would cause abnormal losses. A milliwatts loss test conducted on the arresters is probably \\\
the most reliable indication of moisture, contamination, or cracks in the arrester. An increase in the milliwatts loss is evidence of a change in the \\\
characteristics and evidence that the arrester is deteriorating.  One or more arresters tested during this project have elevated milliwatts test values. \\\
Recommend replacing these devices. Acceptance testing should be performed on replacement arresters to ensure they are in good condition and establish a \\\
comparison base-line value for future maintenance testing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

RELAYS_Comments: 

'RELAYS
Relay was adjusted to provided settings. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Relay was adjusted to maximum adjustment. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Relay Calibration
Necessary adjustments to calibrate this relay to the as found settings were performed. These settings should be compared to coordination study settings \\\
if available. \\\  
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Relays-Coordination
The protective relays were calibrated at the as-found settings.  Recommend a protective device coordination study be performed to determine if these \\\
are the optimum settings for proper system operation.  Please contact our office for assistance in the development of this engineering study. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Relays-Recommendations
Protective relays should be inspected, calibrated (if necessary), cleaned, contacts burnished (as necessary), every 1 to 5 years based on operating \\\
environment and application. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

RELAYS_Deficiencies: 

'RELAYS

TRIP TIMING ----- Relay operates outside the manufacturer''s recommended trip curve. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

TRIP OPERATION ----- Relay trip test failed to operate breaker.  Requires immediate attention. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Relay cover is damaged and requires replacement. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Ground Fault-Neutral Ground
The neutral feeder conductors are grounded downstream of the neutral disconnect link. Neutral grounds will desensitize the ground fault system \\\
and may cause nuisance tripping. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

ROTATING MACHINERY_Comments: 

'ROTATING MACHINES

'Gen-IR Test OK
An insulation resistance test was performed on your generator. A constant voltage was applied to the windings and readings taken every minute for \\\
ten minutes. A polarization index was calculated which provided a quantitative appraisal of the insulation condition with respect to moisture and \\\
other contaminants. The generator tested passed the insulation resistance test. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Motors-IR Test OK
An insulation resistance test was performed on your motor. A constant voltage was applied to the windings and readings taken every minute \\\
for ten minutes. A polarization index was calculated which provided a quantitative appraisal of the insulation condition with respect to \\\
moisture and other contaminants. The motor tested passed the insulation resistance test. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

ROTATING MACHINERY_Deficiencies: 

'ROTATING MACHINES
'Gen-IR Test Problem
An insulation resistance test was performed on your generator. A constant voltage was applied to the windings and readings taken every minute \\\
for ten minutes. A polarization index was calculated which provided a quantitative appraisal of the insulation condition with respect to moisture \\\ 
and other contaminants. The polarization index number obtained for the generator under test indicated deteriorated insulation. We recommend \\\
re-testing in one year, any significant decrease in the insulation resistance would warrant further investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Motors-IR Test Problem
An insulation resistance test was performed on your motor. A constant voltage was applied to the windings and readings taken every minute for \\\
ten minutes. A polarization index was calculated which provided a quantitative appraisal of the insulation condition with respect to moisture \\\
and other contaminants. The polarization index number obtained for the motor under test indicated deteriorated insulation. We recommend \\\
re-testing in one year, any significant decrease in the insulation resistance would warrant further investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

SWITCHBOARDS_Comments: 

:END

SWITCHBOARDS_Deficiencies: 
 
:END

SWITCHBOARDS/SWITCHGEAR_Comments: 

'SWITCHGEAR/SWITCHBOARDS

:END

SWITCHBOARDS/SWITCHGEAR_Deficiencies: 

'SWITCHGEAR/SWITCHBOARDS

:END

TRANSFER SWITCHES_Comments: 

:END

TRANSFER SWITCHES_Deficiencies: 

:END

TRANSFORMERS_Comments: 

'BUSHINGS

'Bushings-High MW Losses
Hot-collar tests on bushings are highly susceptible to surface contamination and it can be difficult to rid bushings of all surface contamination \\\ 
in the field. Use of cleaners and heat guns can be used to improve the bushing condition for testing purposes. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'TRANSFORMERS

'GENERAL
Fans would not operate in manual mode.  Operating temperature acceptable. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Cooling Fans
COOLING FANS ----- The cooling fans were not on during the inspection. Operating temperatures were moderate. Investigate and repair fan operation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Transformer tank pressure was not found at a positive pressure. Add nitrogen.  Inspect for leaks. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Transformer gas blanket contained oxygen content greater than 3%. Purged to less than 3% with dry nitrogen. Acceptable as left. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Transformer Tests Overview
Insulation power factor, turns ratio, winding resistance, and insulation resistance tests were performed to determine the electrical integrity \\\
of the transformers windings and insulation system. Test results were compared to similar transformers and industry recommended standards as \\\
applicable. This equipment was found to meet applicable NETA standards. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'New PF
The transformers tested passed the insulation power factor, turns ratio, winding resistance and insulation resistance tests. The power factor \\\
values of CL, CH and CHL are below 0.5%, which is the recommended limit by Doble Engineering for new oil filled power transformers above 500 KVA. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Pressure/Vacuum System
P/V SYSTEM ----- The purpose of the pressure/vacuum system is to permit monitoring and purging of the nitrogen blanket. The gas blanket should contain low \\\
oxygen content to aid in reducing oxidation of the insulating liquid and preventing a combustible atmosphere. With a P/V system installed, \\\
the gas blanket can be monitored for oxygen content and total combustible gases. If the oxygen content is too high, it can be purged to less \\\
than 3% with dry nitrogen. Sampling for total combustible gases is important for detecting internal transformer faults. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Purged N2 Blanket
PURGED GAS BLANKET ----- The nitrogen blanket was found with an elevated oxygen content. The nitrogen blanket was purged to less than 3% to reduce oxidation of \\\
the fluid and reduce the chance of establishing a combustible condition within the transformer tank. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

TRANSFORMERS_Deficiencies: 

'BUSHINGS

BUSHING LEAK ----- A fluid leak from one or more bushings was noted during this project. Recommend scheduling an outage to make necessary repairs \\\
to eliminate the leak. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHING HIGH LOSSES ----- Bushing tests performed on this equipment indicates higher than normal milliwatts loss on the one or more bushing. \\\
Compiled test data suggests the milliwatts losses measured on this bushing exceeded the values normally expected. Also of concern is a \\\
comparison of this bushing to other bushings on this and other similar equipment. Abnormally high milliwatts test values most likely reflect \\\
contamination within the bushing which has entered through cracks in the porcelain or a defective seal. Recommend replacing this bushing at \\\
your next convenient opportunity. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHINGS HIGH LOSSES ----- A significant deviation in milliwatts-losses was noted between similar bushings. Normal acceptable deviations between similar bushings is \\\
1.25 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHING HIGH LOSSES ----- A comparison of the watts-loss test results indicates a higher than normal test result on one ore more bushings compared to other similar \\\
bushings and/or industry data on similar bushings.  The following conditions should be considered in the overall evaluation of these bushings: \\\ 
(1) Hot-collar tests on bushings are highly susceptible to surface contamination and it can be difficult to rid bushings of all surface \\\
contamination in the field. Use of cleaners and heat guns can be used to improve the bushing condition for testing purposes,  (2) A deviation \\\
in milliwatts-losses greater than 200% (2.3 mW) exists between bushings. Normal acceptable deviations between similar bushings is 1.25 mW \\\
(Doble Engineering Test-Data Reference Book),  (3) Serious concern is given for bushings with hot-collar test results greater than 6 mW \\\
(Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHING HIGH POWER FACTOR ----- Power factor test conducted on the bushings of this equipment indicates a problem in one or more bushings. A power factor \\\
test is probably the most reliable indication of moisture, contamination or cracks in a bushing. Power factor of sealed bushings is low \\\
initially and remains low in service if the bushing is in good condition. An increase in power factor or a poor reading is evidence of a change \\\
in the characteristics and evidence that a potentially damaging condition is developing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHINGS DIRTY ----- The equipment bushings are dirty and require cleaning. Bushing surface contamination often results in noisy substations, \\\
damage to insulating surfaces, partial discharge, tracking flashover and eventually loss of power. It is possible to prevent bushings from \\\
flashing over by periodic cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Serious concern is given for bushings with hot-collar test results greater than 6 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Insulation power factor tests conducted on the bushings indicates possible problems with one or more bushings. An insulation power factor test \\\
is probably the most reliable indication of moisture, contamination or cracks in a bushing. An increase in power factor or a poor reading is \\\
evidence of a change in the characteristics and evidence that a potentially damaging condition is developing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

A significant deviation in milliwatts-losses was noted between similar bushings. Normal acceptable deviations between similar bushings is \\\
1.25 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Serious concern is given for bushings with hot-collar test results greater than 6 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Additional recommendations for the deteriorated bushing(s) can be provided after the next set of tests are performed and results are evaluated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

While it is impossible to guarantee the continued reliable operation of any device based on test data alone, the test results of this bushing \\\
cause concern for its continued reliable operation. The test results do not necessarily indicate the bushing will fail in the immediate future \\\
but they do deviate from the norm enough to cause concern and require additional follow up investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Without regard to costs, the most conservative action would be to replace this bushing. However, based on the test results and the above \\\
evaluation criteria, the more reasonable response would be to monitor the bushing and perform additional tests within the next 12 months. \\\
These tests are performed to validate the test results recently obtained and also permit additional investigative tests to be performed \\\
to better identify the potential defect of this bushing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


'TRANSFORMERS

TTR ----- Turns ratio exceeds recommended 0.5% tolerance. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

DIRTY EQUIPMENT ----- Bushings are dirty and require cleaning to avoid potential flashover. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

FAN OPERATION ----- Fans would not operate in manual mode.  Operating temperature is high and damage to insulation is possible. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

DEFECTIVE GUAGE ----- Fluid level gauge was found damaged/defective.  Replace gauge. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

FANS INOP ----- Fans would not operate in manual mode.  Operating temperature acceptable. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

DEFECTIVE GAUGE ----- Temperature gauge was found damaged/defective.  Replace gauge. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

TANK PRESSURE ----- Transformer tank pressure was not found at a positive pressure. \\\ 
Add nitrogen.  Inspect for leaks. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Dry-Type PF-Poor
The transformers tested passed the insulation power factor, turns ratio, winding resistance and insulation resistance tests. The power factor \\\
values of CL, CH and CHL are below the recommended limits by Doble Engineering for new dry-type power transformers above 500 KVA. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'High PF
Power factor, turns ratio, winding resistance, and insulation resistance tests were used to determine the electrical integrity of the \\\
transformer's windings and insulation. Test results were compared to data in our files of similar transformers. Measured test results indicated \\\
higher than normal power factor readings on the one or more windings. Higher than normal power factor readings usually indicate the insulation \\\
to ground is contaminated with moisture and/or dirt, or the insulation is deteriorated as a result of voltage stress and corona damage. \\\
Recommend re-testing in 6 months. Any significant increase in the power factor readings would warrant further investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'High PF/Humidity
The power factor for this transformer is abnormally high. It is uncertain if the power factor is abnormally high due to moisture \\\
(humidity/dew point) or if the high power factor is a result of a high-losses in the material in the transformer. Standard industry data \\\
suggests that an insulation power factor of less than 2% may be more normal. Recommend re-testing this transformer in one year under \\\
more favorable weather conditions. Any significant increase of the insulation power factor over the present level would warrant further \\\
investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

The power factor for this transformer is abnormally high. It is uncertain if the power factor is abnormally high due to moisture \\\
(humidity/dew point) or if the high power factor is a result of a high-losses in the material in the transformer. Standard industry data \\\
suggests that an insulation power factor of less than 1% may be more normal. Recommend re-testing this transformer in one year  under \\\
more favorable weather conditions. Any significant increase of the insulation power factor over the present level would warrant further \\\
investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

The insulation power factor test results are rated deteriorated.  These results should be verified immediately to determine necessary actions. \\\
Elevated insulation power factor may indicate moisture contamination of the cellulose insulation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Bird Nests
Bird nests were noted in the substation structure and/or equipment. Recommend removing nests to improve cooling. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Bushing Leak
During this inspection, a fluid leak at a bushing was noted.  Recommend scheduling an outage to make necessary repairs to eliminate the leak. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Bushings-Dirty
The equipment bushings are dirty and require cleaning. Bushing surface contamination often results in noisy substations, damage to \\\
insulating surfaces, partial discharge, tracking flashover and eventually loss of power. It is possible to prevent bushings from flashing \\\
over by periodic cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Defective Temp Gauge
The transformer temperature gauge is defective.  Recommend replacing defective gauge. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Fluid Level Gauge
The fluid level cannot be determined by the level gauge.  Recommend investigating fluid level and replacing gauge to determine fluid \\\
level during future inspections. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Fluid-Slightly Low
Based on the liquid level gauge, the liquid level of this equipment is slightly low. During a future maintenance project, recommend adding \\\
the necessary liquid to the proper level. A low liquid level condition can result in excessive heating and insulation damage. Adequate liquid \\\
is necessary to permit proper circulation and cooling. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Low liquid
Based on the liquid level gauge, the liquid level of this equipment is below its normal level.  Recommend adding the necessary liquid to \\\
the proper level. A low liquid level condition can result in excessive heating and insulation damage. Adequate liquid is necessary to permit \\\
proper circulation and cooling. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Max temp
The maximum temperature indicator on the temperature gauge indicates this transformer has exceeded the maximum recommended operating temperature. \\\
Top liquid temperature gauges can only provide a general indication of winding temperature because the liquid temperature lags far behind the \\\
winding temperature. If the top liquid temperature gauge is the only temperature indication, a liquid temperature above 80 degrees C for a 65 degrees C rise \\\
transformer or 70 degrees C for a 55 degrees C rise transformer should not be exceeded. A transformer whose nameplate indicates a 65oC rise means that the \\\
transformer winding will experience normal life expectancy if the winding temperature does not exceed 65 degrees C over the ambient or surrounding \\\
temperature. If the ambient temperature is estimated to be 30 degrees C, then the actual temperature of the windings could be as high as 95 degrees C. For a \\\
65oC rise transformer, a sustained winding operating temperature above 95 degrees C or top liquid temperature above 80 degrees C would warrant further \\\
investigation and monitoring. If the transformer insulation is rated for 55 degrees C rise, a sustained winding operating temperature above 85 degrees C or \\\
top liquid temperature above 70 degrees C would warrant further investigation and monitoring. \\\ 
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'N2-High O2
The nitrogen gas blanket of this transformer was found to contain a high level of oxygen. Oxygen in the gas space can contribute to fluid \\\
deterioration and provide for a combustible atmosphere inside the transformer.  Recommend purging the gas space to less than 3% oxygen. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Paint
The paint condition of this equipment has deteriorated. Application of paint can protect against severe deterioration caused by \\\
destructive atmospheric elements. Because electrical equipment generate internal heat, this protective coating must permit heat to escape. \\\
Corroded surfaces do not allow proper heat transfer.  Recommend removal of any rust and application of a protective coat of paint. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\ 

'Pressurize Transformer
The transformer is not under a positive pressure.  Recommend adding necessary nitrogen to provide a positive pressure. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Repair Nitrogen Leak
The oxygen content in the nitrogen blanket is elevated.  Recommend performing investigation to locate leaks in the transformer gas \\\
blanket. An outage to replace gaskets may be necessary. Purge the nitrogen blanket to less than 3% to reduce oxidation of \\\
the fluid. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Repair P/V Gauge
The pressure/vacuum gauge is damaged. Replace gauge. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Sample Valve Seep
A seep of fluid from the sample valve was noted during this inspection. Valve should be monitored to determine if the seep continues. Repair valve \\\
if necessary. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Tank Grounds
A transformer tank ground was not visible during inspection. Recommend installing adequate tank ground. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Valve Leak-Monitor
HVA small leak at the sample valve was noted during this inspection. Sample valve should be monitored to determine if the leak continues. Re-packing of the existing \\\
valve or replacement may be necessary. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Valve-Repair/Replace
Recommend repairing the leak noted at the sample valve. Re-packing of the existing valve or replacement is necessary. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'X0 Bushing
X0 BUSHING NOT GROUNDED ----- The X0 bushing on this transformer is not grounded. There are three major problems in operating an ungrounded system \\\
Transient overvoltages to ground may puncture the insulation.  Locating a ground fault may be difficult. Destructive arcing can occur if a second \\\
ground fault occurs before the first fault is cleared \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

VIRTUAL FRONT PANEL_Comments: 

:END

VIRTUAL FRONT PANEL_Deficiencies:

:END

DefaultComments:


'INSTRUCTIONS:
'User to select best comment description from list below. On multi-line comments, user MUST \\\
'select the first line of the selection to include the entire selection.


'SEPARATOR LINE
'Use to separate manually entered statements.
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


'GENERAL COMMENTS

Based on the inspections, maintenance, calibration, and test results performed, this equipment was found to meet applicable NETA standards. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Some test results for this equipment are below applicable NETA standards.  Equipment with test results below NETA recommended values have an increased \\\
chance of in service failure. Based on historical test data for this equipment, the test results remain relatively constant. The customer should schedule \\\
this equipment for replacement or refurbishment. Recommend close monitoring of this equipment and increased test periods until the equipment is replaced \\\
or refurbished. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

INSULATION RESISTANCE:  As described in the Deficiencies and Recommendations Section, insulation resistance test results do not meet NETA minimum \\\
recommended values.  Normally, equipment with test values not meeting the minimum NETA recommendations should be considered for repair or replacement \\\
because it poses increased risk of in-service failure. However, after a review of historical test data for this equipment, it is apparent that the \\\
insulation resistance test results remain comparable with previous values. Based on the reduced but stable test results obtained and the current physical \\\
condition of this equipment, it is logical to conclude that this equipment poses only a slightly increased risk of in-service failure at this time. At the \\\
customers discretion, it would be reasonable for this equipment to remain in service if closely monitored through a program that includes annual inspections, \\\
testing, and maintenance. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

CONTACT RESISTANCE:  As described in the Deficiencies and Recommendations Section, the contact resistance test values do not strictly adhere to the NETA \\\
recommendations for maximum deviation of contact resistance between poles or similar devices. However, since all contact resistance test values were found to \\\
be significantly low in magnitude, this broader range of values is considered acceptable. Thermographic inspection during normal loading can detect problems \\\
associated with high contact resistance because high contact resistance results in abnormally high equipment or component temperatures. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


See Engineering Report for additional comments and recommendations. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

A more detailed course of corrective action can be presented after the next set of tests are performed and the results are evaluated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


Insulating members were found dirty and required cleaning. Insulating members were cleaned. Acceptable as left. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Equipment is dirty and requires cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Compartment heaters are not on. Compartment is subject to condensation. Repair heaters. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Compartment contains no heaters.  Compartment is subject to condensation.  Recommend installing necessary heaters to eliminate condensation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Equipment shows signs of physical damage.  Recommend repairing/replacing damaged components. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Switchboard, cables, bus and support insulators required cleaning.  Any dirt or foreign material that interfered with cooling or that reduced \\\
clearances to ground was removed. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


:END


'>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
'>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
'>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



DefaultDeficiencies:


'INSTRUCTIONS:
'User to select best deficiency description from list below. On multi-line deficiencies, user MUST \\\
'select the first line of the selection to include the entire selection.

'SEPARATOR LINE
'Use to separate manually entered statements.
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'GENERAL DEFICIENCIES
DANGER. DO NOT RETURN THIS EQUIPMENT TO SERVICE WITHOUT REPAIRS. This equipment failed the electrical test(s) performed. \\\
This equipment should not be returned to service. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Some test results for this equipment are below applicable NETA standards.  Equipment with test results below NETA recommended values have an increased \\\
chance of in service failure. Based on historical test data for this equipment, the test results remain relatively constant. The customer should schedule \\\
this equipment for replacement or refurbishment. Recommend close monitoring of this equipment and increased test periods until the equipment is replaced \\\
or refurbished. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

See Engineering Report for additional comments and recommendations. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

A more detailed course of corrective action can be presented after the next set of tests are performed and the results are evaluated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Insulating members are dirty and require cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Equipment was found not properly anchored. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Equipment was found without proper ground connections. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

See Engineering Report for additional comments and recommendations. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Equipment was found extremely dirty and requires extensive cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Insulating members are dirty and require cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


'GENERAL TESTS

'CONTACT RESISTANCE
CONTACT RESISTANCE ----- The contact resistance test values deviate by more than 50%.  Recommend investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

HIGH CONTACT RESISTANCE ----- Contact resistance test results are higher than other similar equipment.  Recommend investigation. \\\
Perform infrared inspection to determine if excessive heating is a problem. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'INSULATION POWER FACTOR
HIGH INSULATION POWER FACTOR ----- Insulation power factor test results are rated deteriorated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

HIGH WATTS LOSS ----- Watts loss test results are rated deteriorated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


'INSULATION RESISTANCE
LOW PI ----- The polarization index ratio is rated deteriorated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

LOW DAR ----- The dielectric absorption ratio is rated deteriorated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

LOW INSULATION RESISTANCE ----- Insulation resistance test results are below the recommended value. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


:END



Condition:

Acceptable
Dirty
Deteriorated
Not Applicable
Not Inspected
New
Poor
See Comment
See Deficiency
:END


Lube:

Cleaned
Cleaned/Lubed
Lubed
Not Applicable
:END


Manufacturer:
ABB
ABB Power T&D Co
Aetna
Allen Bradley
Allis-Chalmers
Alstom T&D - Balteau
ASEA
Brown-Boveri
Challenger
Cooper Power Systems
Craig Electric
Cutler-Hammer
Delta Star, Inc
Eaton
Federal Pacific Electric
Feranti Packard
Furnas
General Cable
General Electric
GS - Hevi Duty
Hevi Duty Electric
Howard Industries, Inc
Line Material
McGraw Edison
No Nameplate
Okonite
Olsun
S&C Electric Company
Schneider Canada
Siemens
Siemens Energy & Automation
Solar
Sorgel-Square D
Square D
Sylvania
Topaz DPP Division
Unknown
Wagner Electric
Westinghouse
:END

AmpRating:
15
20
25
30
35
40
45
50
60
70
80
90
100
110
125
150
175
200
225
250
300
350
400
450
500
600
700
800
1000
1200
1600
2000
2500
3000
4000
5000
6000
:END

NomVolts:
120
120/240
208Y/120
240
347
480Y/277
480
600Y/347
600
:END

InstallMethod:

cable tray
duct bank
:END
           џџџ 0                            mE#         $2                џџџ (   (   (   (                 mE#On Initial Update     mE#	On Updateџэ
if GetVar("PdbDeviceGuid")  = "zk00037r@\]d2/Џ2" then 
    if GetVar("Manufacturer_List") <> Empty and GetVar("Mfr") = Empty then 
        call SetVar("Mfr", GetVar("Manufacturer_List") )
        call SetVar("Manufacturer_List",Empty)
        call SetVar("SerNo", GetVar("SerialNo") )
        call SetVar("VA_1", GetVar("kva1") )
        call SetVar("VA_2", GetVar("kva2") )
        call SetVar("COOLANT", GetVar("COOLANT") )
        call SetVar("Phases", GetVar("Phase") )
        call SetVar("TankType", GetVar("TYPE") )
        call SetVar("Type", GetVar("CLASS") )
        call SetVar("WdgMaterial", GetVar("windmaterial") )
        call SetVar("Weight",Empty)
        call SetVar("Bil", CStr(GetVar("KVPri") ) )
        call SetVar("Weight", GetVar("TotalWeight") )
        call SetVar("Gallons", GetVar("maintankcap") )

        '
        ' ERS to provide mapping
        '
        if .TYPE = "Sealed Gas Filled Dry Type" then .TankType = "Sealed"




    end if 
end if      mE#On Page Set Focus     mE#On Form Exit     mE#On Timer 	    mE#Global Functions && VariablesRfunction MakeSQLSafe(text)
    MakeSQLSafe=Replace(text,"'","''")
end Function
    mE#DataџџџNb BATTERIES_Comments: 

'DC BATTERIES/CHARGERS

'Battery Bank - General
Battery banks are the last barrier between critical equipment operation and catastrophe. Good maintenance practices include load testing to \\\
ensure the batteries can provide the required current if the need arises. A thermographic inspection during load tests can reveal dangerous \\\
conditions. Connections not providing sufficiently low contact resistance resulting from corrosion and improper torque are easily identified \\\
under loaded conditions using thermography. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\
:END

BATTERIES_Deficiencies: 

'DC BATTERIES/CHARGERS

SEDIMENT/CORROSION ----- Batteries show signs of sediment and/or corrosion. Recommend performing preventive maintenance and testing to determine the condition of the \\\
battery system. Recommend cleaning all connections, testing for connection resistance, electrolyte specific gravity, cell resistance, charger \\\
float and equalize settings, and perform a load test to determine battery system capacity and overall condition. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

CORROSION ----- The batteries show signs of corrosion. Recommend performing cleaning and connection resistance testing of the battery bank connections. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

SPECIFIC GRAVITY ----- Batteries tested have specific gravity readings of the electrolyte that deviate significantly from expected values. Recommend replacing these \\\
batteries. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

CABLES/BUS DUCT_Comments: 

'CABLES

'Cable Test Period
Cables should be inspected annually and tested every 3 to 5 years. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Cables- New OK
High potential tests were used to determine the electrical integrity of the cable systems tested. Cable systems tested included the terminations. \\\
The main objective of this test is to make sure the dielectric of the cables has not been damaged and to determine the integrity of the stress \\\
relief devices. Cables tested passed the high potential test. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Cables-Existing OK
High potential tests were used to determine the electrical integrity of the cable systems tested. Cable systems tested included the terminations. \\\ 
The main objective of this test is to make sure the dielectric of the cables has not been damaged and to determine the integrity of the stress \\\
relief devices. Cables tested passed the high potential test. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'BUSWAY

:END

CABLES/BUS DUCT_Deficiencies: 

'BUSWAY

'CABLES

:END

CALIBRATION_Comments: 

:END

CALIBRATION_Deficiencies: 

:END

CIRCUIT BREAKER_Comments: 

'CIRCUIT BREAKERS

'LOW VOLTAGE

'CB-Static OK
High Current, contact resistance, and insulation resistance tests were used to determine the electrical integrity of the circuit breakers tested. \\\
High current testing, which is the only reliable method of checking static overcurrent trip devices, was used to verify the complete tripping \\\
system. Circuit breakers tested passed the high current, contact resistance and insulation resistance tests. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'MEDIUM/HIGH VOLTAGE

'MV CB Test Period
Medium Voltage Circuit Breakers should be inspected, cleaned, lubricated (as necessary), tested, and function checked every 1 to 3 years based on \\\
operating environment and application. MV circuit breakers should have the following tests performed: insulation resistance, insulation power \\\
factor, and contact resistance. Breakers require exercising to assure proper operation and distribution of lubricants. Breakers should be \\\
exercised annually (minimum recommendation). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'MVCB-Tests OK
Power factor, contact resistance and insulation resistance tests were used to determine the electrical integrity of the medium voltage air \\\
circuit breakers tested. Test results were compared to data in our files of similar breakers. Medium voltage breakers tested passed the \\\
insulation power factor, contact resistance and insulation resistance tests. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'OCB-Tests OK
Power factor/Tank-Loss Index, contact resistance and insulation resistance tests were used to determine the electrical integrity of the medium \\\
voltage oil circuit breakers tested. Test results were compared to data in our files of similar breakers. Medium voltage breakers tested passed \\\
the insulation power factor, contact resistance and insulation resistance tests. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

CIRCUIT BREAKER_Deficiencies: 

'CIRCUIT BREAKERS

LOW INSULATION RESISTANCE ----- The insulation resistance test values were less than recommended by NETA MTS standards.  Recommend additional investigation to determine \\\
cause of deteriorated insulation resistance.  Replacement of breaker components such as pole pieces may be necessary to improve insulation resistance. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


'LOW VOLTAGE

'CB-Replace
This circuit breaker did not trip within acceptable time limits when tested by primary current injection. Recommend replacing this breaker. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'CB-Replace Trip Unit
This circuit breaker did not trip within acceptable time limits when tested by primary current injection. Recommend replacing the trip unit \\\
and re-testing following the repairs. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

INJECTION TIMING TEST ----- The long time element on one or more poles of this breaker does not conform to the manufacturer''s published time curves.  The principal \\\
purpose of field testing is not to determine if the breaker exactly meets published time curves, but rather to determine if the circuit breaker is furnishing the \\\
protection for which it was installed.  A circuit breaker that trips in less than the minimum time shown by the trip time curves will furnish protection at a lower \\\
current level.  Unless this results in nuisance tripping or coordination problems, this condition is may be considered acceptable. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

INJECTION TIMING TEST ----- Instantaneous element on one or more poles of this breaker does not conform to the manufacturer''s published time curves.  The principal purpose \\\
of field testing is not to determine if the breaker exactly meets published time curves, but rather to determine if the circuit breaker is furnishing the protection for which \\\
it was installed.  A circuit breaker that trips on less current than the minimum current shown on the manufacturer''s published time curves will furnish protection at a lower \\\
current level.  Unless this results in nuisance tripping or coordination problems with the circuit, this condition may be considered acceptable. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

INJECTION TIMING TEST ----- Circuit breaker trips outside the manufacturer''s recommended trip curve. Investigate and/or repair/replace trip unit. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

INJECTION TIMING TEST ----- Circuit breaker failed to trip.  Replace breaker/trip unit. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

REPLACE DEFECTIVE/ANTIQUATED TRIP UNIT ----- This circuit breaker did not trip within acceptable time limits when tested by primary current injection.  The trip unit \\\
in this type of breaker is an electro-mechanical series overcurrent device.  These devices have sucker discs that become sticky, mechanisms \\\
that corrode, small air or oil dashpot orifices that become clogged, rubber diaphragms that rupture, etc.  We recommend retrofitting this breaker \\\
with a static trip device with electronic current sensing and trip timing.  The static trip units are reliable devices, with precise, repeatable tripping characteristics. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

REPLACE DEFECTIVE TRIP UNIT ----- This circuit breaker did not trip within acceptable time limits when tested by primary current injection.  Recommend replacing the \\\
trip unit and re-testing following the repairs. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


'MEDIUM/HIGH VOLTAGE

'MVCB-Moisture/Investigate
Power factor insulation tests on this medium voltage air circuit breaker indicates deteriorated insulation. The bushing power factor test at \\\
2,500 volts (breaker open, opposite bushing guarded) indicated power factors greater than six percent. Data in our files suggests that a power \\\
factor of less than 6% may be more normal. Our test results indicate moisture has been absorbed by the circuit breaker insulation. \\\
The following is recommended:  (1) All openings to the breaker cubicle should be eliminated. Particular attention should be given to  openings \\\
from conduits, cable ducts or trenches. (2) Most cubicles require some venting, however, certain old breaker designs include top and bottom \\\
openings, where a limited venting would be more suitable.  (3) Heaters should be mounted at the cubicle floor level on either side of the breaker. \\\	
Approximately 1500 watts of heat is required to reduce moisture in the breaker.  Circuit breaker should be removed from its cubicle in dried. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'CB-IR/Deteriorated
The insulation resistance test results are rated deteriorated. Additional investigation is necessary to determine cause of deteriorated \\\
insulation resistance. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'MVCB-Moisture/Investigate
Insulation power factor insulation tests on this medium voltage air circuit breaker indicates deteriorated insulation. The bushing power factor test \\\
results (breaker open, opposite bushing guarded) indicates power factors greater than 6%. Power factor test values of less than 6% may be more \\\
normal. High power factor test results are commonly caused from moisture absoption of the equipment insulation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'MVCB-Moisture/Investigate
Insulation power factor tests on this medium voltage air circuit breaker indicates deteriorated insulation. The bushing power factor test results \\\
(breaker open, opposite bushing guarded) are elevated.  Test results indicate moisture has been absorbed by the circuit breaker insulation. \\\
Recommend the following actions:  (1) All openings to the breaker cubicle should be eliminated. Particular attention should be given to openings \\\
from conduits, cable ducts or trenches. Most cubicles require some venting, however, certain old breaker designs include top and bottom openings, \\\
where a limited venting would be more suitable.  (2) Heaters should be mounted at the cubicle floor level on either side of the breaker. \\\
(3) Approximately 1500 watts of heat is required to reduce moisture in the breaker. (4) Circuit breaker should be removed from its cubicle \\\ 
in dried. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

The following actions are recommended: (1) All openings to the breaker cubicle should be eliminated. Particular attention should be given to openings\\\
from conduits, cable ducts or trenches. Most cubicles require some venting, however, certain old breaker designs include top and bottom openings, where \\\
a limited venting would be more suitable.  (2) Heaters should be mounted at the cubicle floor level on either side of the breaker.  (3) Approximately \\\
1500 watts of heat is required to reduce moisture in the breaker. (4) Circuit breaker should be removed from its cubicle and dried. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

COORDINATION DATA_Comments: 

:END

COORDINATION DATA_Deficiencies: 

:END

DISCONNECTS/SWITCHES_Comments: 

'SWITCHES

'LOW VOLTAGE SWITCHES
Contact resistance measurements and insulation resistance tests were performed on the low voltage switches.  Insulation resistance tests \\\
are used to determine the quality or condition of the switch insulation.  Contact resistance measurements are used to determine the quality of \\\
the switch contacts.  The contact resistance and insulation resistance test results for this equipment was found to meet applicable NETA standards. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

DISCONNECTS/SWITCHES_Deficiencies: 

'SWITCHES


MECHANICAL OPERATION ----- Switch does not operate reliably.  Recommend exercising and lubricating switch. Additional maintenance may be required. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

EQUIPMENT FIELD SURVEY_Comments: 

:END 

EQUIPMENT FIELD SURVEY_Deficiencies: 

:END 

GENERATORS_Comments: 

:END

GENERATORS_Deficiencies: 

:END

GROUND FAULT_Comments: 

'GROUND FAULT PROTECTION

'Ground Fault-Purpose
The purpose of testing a ground fault system is to ensure continued correct operation of the ground fault protective equipment. Deterioration of \\\
electrical equipment is a normal process. Since the deterioration of electrical systems can lead to equipment malfunction and electrical failure, \\\
the National Fire Protection Association in NFPA 70B recommends periodic testing of ground fault systems to confirm operation of the relays. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Ground Fault-Test OK
Ground fault relays were tested by injecting current into the relays and measuring for pickup and tripping time values. All relays tested were \\\
found operational requiring minor cleaning and no calibration adjustments. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Ground Test-ok
Ground fault relays were tested by injecting current into the relays and measuring for pickup and tripping time values. All relays tested were \\\
found operational requiring minor cleaning and no calibration adjustments. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

GROUND FAULT_Deficiencies: 

'GROUND FAULT PROTECTION

:END

GROUND MAT (EARTH) GROUNDING TESTS_Comments: 

'GROUNDING SYSTEMS

'Ground Test-Purpose
The purpose of ground resistance tests is to determine the effectiveness of the grounding electrode system which is used to protect \\\
personnel and equipment. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Adequate distance from the ground system under test was not available because of geographic limitations. Alternative ground testing was performed. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

GROUND MAT (EARTH) GROUNDING TESTS_Deficiencies: 

'GROUNDING SYSTEMS

HIGH RESISTANCE ----- Ground resistance measurement exceeds specification. Ground system requires modification to meet specification. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

HIGH RESISTANCE ----- Point-to-point resistance value exceeds 0.5 ohms. Investigate cause of high test result. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

The soil conditions should be reviewed to determine the possible causes for the unacceptable results and that corrective actions be implemented. \\\
A retest of the system should be completed after modifications to ensure the desired ground resistance is achieved. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

The ground resistance was found to be unacceptable in accordance with the project specification requirement of 5 ohms.  Supplemental grounding \\\
electrodes or other means will be required to meet the specifications requirements.  This can be accomplished by many alternatives including extending \\\
electrode deeper, adding electrodes and using alternate types to increase the contact surface area with the earth.  The choice of methods depends on the \\\
coil soil conditions that are causing an unsatisfactory test result. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

The neutral feeder conductors are grounded downstream of the neutral disconnect link.  Neutral grounds will desensitize the ground fault system \\\
and may cause nuisance tripping. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Ground Test-Recommendations
The soil conditions should be reviewed to determine the possible causes for the unacceptable results and that corrective actions be implemented. \\\
A retest of the system should be completed after modifications to ensure the desired ground resistance is achieved. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

INFRARED_Comments: 

'INFRARED/THERMOGRAPHIC INSPECTION

Following all repairs, a follow-up thermographic inspection should be scheduled to assure the deficiencies have been adequately repaired. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

INFRARED_Deficiencies:

'INFRARED/THERMOGRAPHIC INSPECTION

ELEVATED TEMPERATURE ----- Contacts of this device exhibit elevated temperature.  Recommend additional investigation and repairs. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

ABNORMAL HEATING ----- Abnormal heating of this equipment was detected.  Recommend additional investigation and make necessary repairs. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

ELEVATED CONNECTION TEMPERATURE ----- Connection(s) of this equipment has/have elevated temperatures.  Recommend cleaning and checking connections tight. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Many of the deficiencies noted involve fused disconnect switches.  These deficiencies usually result from one or more of the following: \\\
(1) Loose or corroded connections.  These connections should be disassembled, cleaned, and tightened.  Be aware that there are sometimes \\\
hidden connections behind the switch that require disassembly and cleaning,  (2) Deteriorated switch mechanisms.  Pivot points and blade \\\
connections sometimes wear and create a high resistance path.  Disassembly, cleaning, or replacement is sometimes necessary,  (3) Fuse clip \\\
deterioration.  Over time, fuse clips can be come loose and create a high resistance path.  Fuse clips that are worn and no longer provide \\\
the proper retaining tension should be replaced. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Other common deficiencies involve motor starter overload devices.  Recommendations for overload overheating are: (1) Verify overloads are a \\\
matched set and properly sized,  (2) verify motor currents are balanced,  (3) Verify connections are clean and tight. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

INSTRUMENT TRANSFORMERS_Comments: 

'INSTRUMENT TRANSFORMERS

:END

INSTRUMENT TRANSFORMERS_Deficiencies: 

'INSTRUMENT TRANSFORMERS

:END

INSULATION FLUID_Comments: 

'INSULATING FLUID

:END

INSULATION FLUID_Deficiencies: 

'INSULATING FLUID

Fluid seepage/leaks were noted at the equipment bushings. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Fluid seepage/leaks were noted at the equipment covers. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Fluid seepage/leaks were noted at the drain valve. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Fluid seepage/leaks were noted at the sample valve. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Fluid seepage/leaks were noted at the radiators. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Low Liquid
LIQUID LOW ----- Based on the liquid level gauge, the liquid level of this equipment is below its normal level. Recommend adding the necessary liquid to \\\
the proper level. A low liquid level condition can result in excessive heating and insulation damage. Adequate liquid is necessary to permit \\\
proper circulation and cooling. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Liquid Level
The fluid level cannot be determined by the level gauge. Recommend investigating fluid level and replacing gauge to determine fluid level \\\
during future inspections. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Fluid-Slightly Low
Based on the liquid level gauge, the liquid level of this equipment is slightly low. During a future maintenance project, recommend adding the \\\
necessary liquid to the proper level. A low liquid level condition can result in excessive heating and insulation damage. Adequate liquid is \\\
necessary to permit proper circulation and cooling. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

A small leak at the sample valve was noted. Sample valve should be monitored to determine if the leak continues. Re-packing of the existing \\\
valve or replacement may be necessary \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Recommend repairing the leak noted at the sample valve. Re-packing of the existing valve or replacement is necessary \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

FLUID DETERIORATED ----- Insulation fluid is rated deteriorated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

HIGH GASSES ----- Insulating fluid has high dissolved gas content. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

HIGH MOISTURE ----- Insulating fluid has high moisture content. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

FREE WATER ----- Insulating fluid has free water. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

METERS & TRANSDUCERS_Comments: 

'METERS
Verification of the switchboard ammeter and voltmeter accuracy has been accomplished by comparison to test sets maintained by High Voltage \\\
Maintenance Corporation. The accuracy of these test sets are traceable to the National Standards maintained by the National Institute of Standards \\\
in Washington, D.C. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

METERS & TRANSDUCERS_Deficiencies: 

'METERS
Meter is out of calibration. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Meter cover is broken and requires replacement. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Meter is damaged and requires replacement. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

MISCELLANEOUS_Comments: 

'CAPACITORS

:END

MISCELLANEOUS_Deficiencies: 

'CAPACITORS

:END

MOTOR CONTROL CENTERS_Comments: 

'MOTOR CONTROL CENTERS

'MOTOR CONTROL STARTERS

:END

MOTOR CONTROL CENTERS_Deficiencies: 

'MOTOR CONTROL CENTERS

'MOTOR CONTROL STARTERS

:END

POWER FACTOR TESTS_Comments: 

'BUSHINGS

'Bushings-High MW Losses
Hot-collar tests on bushings are highly susceptible to surface contamination and it can be difficult to rid bushings of all surface contamination \\\ 
in the field. Use of cleaners and heat guns can be used to improve the bushing condition for testing purposes. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'SURGE ARRESTERS

'Arresters-ok
Power factor and insulation resistance tests were used to determine the electrical integrity of the lightning arresters tested. The main \\\
objective of this test is to detect contamination by moisture and/or dirt, corroded gap, cracked porcelain, broken shunting resistors, broken \\\
elements, or improper assembly which would cause abnormal losses. Arresters tested passed the insulation power factor and insulation resistance \\\
tests. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

POWER FACTOR TESTS_Deficiencies: 

'BUSHINGS

BUSHING LEAK ----- A fluid leak from one or more bushings was noted during this project. Recommend scheduling an outage to make necessary repairs \\\
to eliminate the leak. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHING HIGH LOSSES ----- Bushing tests performed on this equipment indicates higher than normal milliwatts loss on the one or more bushing. \\\
Compiled test data suggests the milliwatts losses measured on this bushing exceeded the values normally expected. Also of concern is a \\\
comparison of this bushing to other bushings on this and other similar equipment. Abnormally high milliwatts test values most likely reflect \\\
contamination within the bushing which has entered through cracks in the porcelain or a defective seal. Recommend replacing this bushing at \\\
your next convenient opportunity. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHINGS HIGH LOSSES ----- A significant deviation in milliwatts-losses was noted between similar bushings. Normal acceptable deviations between similar bushings is \\\
1.25 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHING HIGH LOSSES ----- A comparison of the watts-loss test results indicates a higher than normal test result on one ore more bushings compared to other similar \\\
bushings and/or industry data on similar bushings.  The following conditions should be considered in the overall evaluation of these bushings: \\\ 
(1) Hot-collar tests on bushings are highly susceptible to surface contamination and it can be difficult to rid bushings of all surface \\\
contamination in the field. Use of cleaners and heat guns can be used to improve the bushing condition for testing purposes,  (2) A deviation \\\
in milliwatts-losses greater than 200% (2.3 mW) exists between bushings. Normal acceptable deviations between similar bushings is 1.25 mW \\\
(Doble Engineering Test-Data Reference Book),  (3) Serious concern is given for bushings with hot-collar test results greater than 6 mW \\\
(Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHING HIGH POWER FACTOR ----- Power factor test conducted on the bushings of this equipment indicates a problem in one or more bushings. A power factor \\\
test is probably the most reliable indication of moisture, contamination or cracks in a bushing. Power factor of sealed bushings is low \\\
initially and remains low in service if the bushing is in good condition. An increase in power factor or a poor reading is evidence of a change \\\
in the characteristics and evidence that a potentially damaging condition is developing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHINGS DIRTY ----- The equipment bushings are dirty and require cleaning. Bushing surface contamination often results in noisy substations, \\\
damage to insulating surfaces, partial discharge, tracking flashover and eventually loss of power. It is possible to prevent bushings from \\\
flashing over by periodic cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Serious concern is given for bushings with hot-collar test results greater than 6 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Insulation power factor tests conducted on the bushings indicates possible problems with one or more bushings. An insulation power factor test \\\
is probably the most reliable indication of moisture, contamination or cracks in a bushing. An increase in power factor or a poor reading is \\\
evidence of a change in the characteristics and evidence that a potentially damaging condition is developing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

A significant deviation in milliwatts-losses was noted between similar bushings. Normal acceptable deviations between similar bushings is \\\
1.25 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Serious concern is given for bushings with hot-collar test results greater than 6 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Additional recommendations for the deteriorated bushing(s) can be provided after the next set of tests are performed and results are evaluated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

While it is impossible to guarantee the continued reliable operation of any device based on test data alone, the test results of this bushing \\\
cause concern for its continued reliable operation. The test results do not necessarily indicate the bushing will fail in the immediate future \\\
but they do deviate from the norm enough to cause concern and require additional follow up investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Without regard to costs, the most conservative action would be to replace this bushing. However, based on the test results and the above \\\
evaluation criteria, the more reasonable response would be to monitor the bushing and perform additional tests within the next 12 months. \\\
These tests are performed to validate the test results recently obtained and also permit additional investigative tests to be performed \\\
to better identify the potential defect of this bushing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'SURGE ARRESTERS
WATTS LOSS DETERIORATED ----- Insulation power factor (watts loss) and insulation resistance tests were used to determine the electrical integrity of the \\\ 
arresters tested. The main objective of these tests is to detect contamination by moisture and/or dirt, corroded gap, cracked porcelain, broken shunting \\\
resistors, broken elements, or improper assembly which would cause abnormal losses. A milliwatts loss test conducted on the arresters is probably \\\
the most reliable indication of moisture, contamination, or cracks in the arrester. An increase in the milliwatts loss is evidence of a change in the \\\
characteristics and evidence that the arrester is deteriorating.  One or more arresters tested during this project have elevated milliwatts test values. \\\
Recommend replacing these devices. Acceptance testing should be performed on replacement arresters to ensure they are in good condition and establish a \\\
comparison base-line value for future maintenance testing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

RELAYS_Comments: 

'RELAYS
Relay was adjusted to provided settings. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Relay was adjusted to maximum adjustment. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Relay Calibration
Necessary adjustments to calibrate this relay to the as found settings were performed. These settings should be compared to coordination study settings \\\
if available. \\\  
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Relays-Coordination
The protective relays were calibrated at the as-found settings.  Recommend a protective device coordination study be performed to determine if these \\\
are the optimum settings for proper system operation.  Please contact our office for assistance in the development of this engineering study. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Relays-Recommendations
Protective relays should be inspected, calibrated (if necessary), cleaned, contacts burnished (as necessary), every 1 to 5 years based on operating \\\
environment and application. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

RELAYS_Deficiencies: 

'RELAYS

TRIP TIMING ----- Relay operates outside the manufacturer''s recommended trip curve. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

TRIP OPERATION ----- Relay trip test failed to operate breaker.  Requires immediate attention. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Relay cover is damaged and requires replacement. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Ground Fault-Neutral Ground
The neutral feeder conductors are grounded downstream of the neutral disconnect link. Neutral grounds will desensitize the ground fault system \\\
and may cause nuisance tripping. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

ROTATING MACHINERY_Comments: 

'ROTATING MACHINES

'Gen-IR Test OK
An insulation resistance test was performed on your generator. A constant voltage was applied to the windings and readings taken every minute for \\\
ten minutes. A polarization index was calculated which provided a quantitative appraisal of the insulation condition with respect to moisture and \\\
other contaminants. The generator tested passed the insulation resistance test. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Motors-IR Test OK
An insulation resistance test was performed on your motor. A constant voltage was applied to the windings and readings taken every minute \\\
for ten minutes. A polarization index was calculated which provided a quantitative appraisal of the insulation condition with respect to \\\
moisture and other contaminants. The motor tested passed the insulation resistance test. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

ROTATING MACHINERY_Deficiencies: 

'ROTATING MACHINES
'Gen-IR Test Problem
An insulation resistance test was performed on your generator. A constant voltage was applied to the windings and readings taken every minute \\\
for ten minutes. A polarization index was calculated which provided a quantitative appraisal of the insulation condition with respect to moisture \\\ 
and other contaminants. The polarization index number obtained for the generator under test indicated deteriorated insulation. We recommend \\\
re-testing in one year, any significant decrease in the insulation resistance would warrant further investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Motors-IR Test Problem
An insulation resistance test was performed on your motor. A constant voltage was applied to the windings and readings taken every minute for \\\
ten minutes. A polarization index was calculated which provided a quantitative appraisal of the insulation condition with respect to moisture \\\
and other contaminants. The polarization index number obtained for the motor under test indicated deteriorated insulation. We recommend \\\
re-testing in one year, any significant decrease in the insulation resistance would warrant further investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

SWITCHBOARDS_Comments: 

:END

SWITCHBOARDS_Deficiencies: 
 
:END

SWITCHBOARDS/SWITCHGEAR_Comments: 

'SWITCHGEAR/SWITCHBOARDS

:END

SWITCHBOARDS/SWITCHGEAR_Deficiencies: 

'SWITCHGEAR/SWITCHBOARDS

:END

TRANSFER SWITCHES_Comments: 

:END

TRANSFER SWITCHES_Deficiencies: 

:END

TRANSFORMERS_Comments: 

'BUSHINGS

'Bushings-High MW Losses
Hot-collar tests on bushings are highly susceptible to surface contamination and it can be difficult to rid bushings of all surface contamination \\\ 
in the field. Use of cleaners and heat guns can be used to improve the bushing condition for testing purposes. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'TRANSFORMERS

'GENERAL
Fans would not operate in manual mode.  Operating temperature acceptable. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Cooling Fans
COOLING FANS ----- The cooling fans were not on during the inspection. Operating temperatures were moderate. Investigate and repair fan operation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Transformer tank pressure was not found at a positive pressure. Add nitrogen.  Inspect for leaks. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Transformer gas blanket contained oxygen content greater than 3%. Purged to less than 3% with dry nitrogen. Acceptable as left. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Transformer Tests Overview
Insulation power factor, turns ratio, winding resistance, and insulation resistance tests were performed to determine the electrical integrity \\\
of the transformers windings and insulation system. Test results were compared to similar transformers and industry recommended standards as \\\
applicable. This equipment was found to meet applicable NETA standards. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'New PF
The transformers tested passed the insulation power factor, turns ratio, winding resistance and insulation resistance tests. The power factor \\\
values of CL, CH and CHL are below 0.5%, which is the recommended limit by Doble Engineering for new oil filled power transformers above 500 KVA. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Pressure/Vacuum System
P/V SYSTEM ----- The purpose of the pressure/vacuum system is to permit monitoring and purging of the nitrogen blanket. The gas blanket should contain low \\\
oxygen content to aid in reducing oxidation of the insulating liquid and preventing a combustible atmosphere. With a P/V system installed, \\\
the gas blanket can be monitored for oxygen content and total combustible gases. If the oxygen content is too high, it can be purged to less \\\
than 3% with dry nitrogen. Sampling for total combustible gases is important for detecting internal transformer faults. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Purged N2 Blanket
PURGED GAS BLANKET ----- The nitrogen blanket was found with an elevated oxygen content. The nitrogen blanket was purged to less than 3% to reduce oxidation of \\\
the fluid and reduce the chance of establishing a combustible condition within the transformer tank. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

TRANSFORMERS_Deficiencies: 

'BUSHINGS

BUSHING LEAK ----- A fluid leak from one or more bushings was noted during this project. Recommend scheduling an outage to make necessary repairs \\\
to eliminate the leak. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHING HIGH LOSSES ----- Bushing tests performed on this equipment indicates higher than normal milliwatts loss on the one or more bushing. \\\
Compiled test data suggests the milliwatts losses measured on this bushing exceeded the values normally expected. Also of concern is a \\\
comparison of this bushing to other bushings on this and other similar equipment. Abnormally high milliwatts test values most likely reflect \\\
contamination within the bushing which has entered through cracks in the porcelain or a defective seal. Recommend replacing this bushing at \\\
your next convenient opportunity. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHINGS HIGH LOSSES ----- A significant deviation in milliwatts-losses was noted between similar bushings. Normal acceptable deviations between similar bushings is \\\
1.25 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHING HIGH LOSSES ----- A comparison of the watts-loss test results indicates a higher than normal test result on one ore more bushings compared to other similar \\\
bushings and/or industry data on similar bushings.  The following conditions should be considered in the overall evaluation of these bushings: \\\ 
(1) Hot-collar tests on bushings are highly susceptible to surface contamination and it can be difficult to rid bushings of all surface \\\
contamination in the field. Use of cleaners and heat guns can be used to improve the bushing condition for testing purposes,  (2) A deviation \\\
in milliwatts-losses greater than 200% (2.3 mW) exists between bushings. Normal acceptable deviations between similar bushings is 1.25 mW \\\
(Doble Engineering Test-Data Reference Book),  (3) Serious concern is given for bushings with hot-collar test results greater than 6 mW \\\
(Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHING HIGH POWER FACTOR ----- Power factor test conducted on the bushings of this equipment indicates a problem in one or more bushings. A power factor \\\
test is probably the most reliable indication of moisture, contamination or cracks in a bushing. Power factor of sealed bushings is low \\\
initially and remains low in service if the bushing is in good condition. An increase in power factor or a poor reading is evidence of a change \\\
in the characteristics and evidence that a potentially damaging condition is developing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

BUSHINGS DIRTY ----- The equipment bushings are dirty and require cleaning. Bushing surface contamination often results in noisy substations, \\\
damage to insulating surfaces, partial discharge, tracking flashover and eventually loss of power. It is possible to prevent bushings from \\\
flashing over by periodic cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Serious concern is given for bushings with hot-collar test results greater than 6 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Insulation power factor tests conducted on the bushings indicates possible problems with one or more bushings. An insulation power factor test \\\
is probably the most reliable indication of moisture, contamination or cracks in a bushing. An increase in power factor or a poor reading is \\\
evidence of a change in the characteristics and evidence that a potentially damaging condition is developing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

A significant deviation in milliwatts-losses was noted between similar bushings. Normal acceptable deviations between similar bushings is \\\
1.25 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Serious concern is given for bushings with hot-collar test results greater than 6 mW (Doble Engineering Test-Data Reference Book). \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Additional recommendations for the deteriorated bushing(s) can be provided after the next set of tests are performed and results are evaluated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

While it is impossible to guarantee the continued reliable operation of any device based on test data alone, the test results of this bushing \\\
cause concern for its continued reliable operation. The test results do not necessarily indicate the bushing will fail in the immediate future \\\
but they do deviate from the norm enough to cause concern and require additional follow up investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Without regard to costs, the most conservative action would be to replace this bushing. However, based on the test results and the above \\\
evaluation criteria, the more reasonable response would be to monitor the bushing and perform additional tests within the next 12 months. \\\
These tests are performed to validate the test results recently obtained and also permit additional investigative tests to be performed \\\
to better identify the potential defect of this bushing. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


'TRANSFORMERS

TTR ----- Turns ratio exceeds recommended 0.5% tolerance. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

DIRTY EQUIPMENT ----- Bushings are dirty and require cleaning to avoid potential flashover. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

FAN OPERATION ----- Fans would not operate in manual mode.  Operating temperature is high and damage to insulation is possible. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

DEFECTIVE GUAGE ----- Fluid level gauge was found damaged/defective.  Replace gauge. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

FANS INOP ----- Fans would not operate in manual mode.  Operating temperature acceptable. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

DEFECTIVE GAUGE ----- Temperature gauge was found damaged/defective.  Replace gauge. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

TANK PRESSURE ----- Transformer tank pressure was not found at a positive pressure. \\\ 
Add nitrogen.  Inspect for leaks. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Dry-Type PF-Poor
The transformers tested passed the insulation power factor, turns ratio, winding resistance and insulation resistance tests. The power factor \\\
values of CL, CH and CHL are below the recommended limits by Doble Engineering for new dry-type power transformers above 500 KVA. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'High PF
Power factor, turns ratio, winding resistance, and insulation resistance tests were used to determine the electrical integrity of the \\\
transformer's windings and insulation. Test results were compared to data in our files of similar transformers. Measured test results indicated \\\
higher than normal power factor readings on the one or more windings. Higher than normal power factor readings usually indicate the insulation \\\
to ground is contaminated with moisture and/or dirt, or the insulation is deteriorated as a result of voltage stress and corona damage. \\\
Recommend re-testing in 6 months. Any significant increase in the power factor readings would warrant further investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'High PF/Humidity
The power factor for this transformer is abnormally high. It is uncertain if the power factor is abnormally high due to moisture \\\
(humidity/dew point) or if the high power factor is a result of a high-losses in the material in the transformer. Standard industry data \\\
suggests that an insulation power factor of less than 2% may be more normal. Recommend re-testing this transformer in one year under \\\
more favorable weather conditions. Any significant increase of the insulation power factor over the present level would warrant further \\\
investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

The power factor for this transformer is abnormally high. It is uncertain if the power factor is abnormally high due to moisture \\\
(humidity/dew point) or if the high power factor is a result of a high-losses in the material in the transformer. Standard industry data \\\
suggests that an insulation power factor of less than 1% may be more normal. Recommend re-testing this transformer in one year  under \\\
more favorable weather conditions. Any significant increase of the insulation power factor over the present level would warrant further \\\
investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

The insulation power factor test results are rated deteriorated.  These results should be verified immediately to determine necessary actions. \\\
Elevated insulation power factor may indicate moisture contamination of the cellulose insulation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Bird Nests
Bird nests were noted in the substation structure and/or equipment. Recommend removing nests to improve cooling. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Bushing Leak
During this inspection, a fluid leak at a bushing was noted.  Recommend scheduling an outage to make necessary repairs to eliminate the leak. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Bushings-Dirty
The equipment bushings are dirty and require cleaning. Bushing surface contamination often results in noisy substations, damage to \\\
insulating surfaces, partial discharge, tracking flashover and eventually loss of power. It is possible to prevent bushings from flashing \\\
over by periodic cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Defective Temp Gauge
The transformer temperature gauge is defective.  Recommend replacing defective gauge. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Fluid Level Gauge
The fluid level cannot be determined by the level gauge.  Recommend investigating fluid level and replacing gauge to determine fluid \\\
level during future inspections. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Fluid-Slightly Low
Based on the liquid level gauge, the liquid level of this equipment is slightly low. During a future maintenance project, recommend adding \\\
the necessary liquid to the proper level. A low liquid level condition can result in excessive heating and insulation damage. Adequate liquid \\\
is necessary to permit proper circulation and cooling. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Low liquid
Based on the liquid level gauge, the liquid level of this equipment is below its normal level.  Recommend adding the necessary liquid to \\\
the proper level. A low liquid level condition can result in excessive heating and insulation damage. Adequate liquid is necessary to permit \\\
proper circulation and cooling. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Max temp
The maximum temperature indicator on the temperature gauge indicates this transformer has exceeded the maximum recommended operating temperature. \\\
Top liquid temperature gauges can only provide a general indication of winding temperature because the liquid temperature lags far behind the \\\
winding temperature. If the top liquid temperature gauge is the only temperature indication, a liquid temperature above 80 degrees C for a 65 degrees C rise \\\
transformer or 70 degrees C for a 55 degrees C rise transformer should not be exceeded. A transformer whose nameplate indicates a 65oC rise means that the \\\
transformer winding will experience normal life expectancy if the winding temperature does not exceed 65 degrees C over the ambient or surrounding \\\
temperature. If the ambient temperature is estimated to be 30 degrees C, then the actual temperature of the windings could be as high as 95 degrees C. For a \\\
65oC rise transformer, a sustained winding operating temperature above 95 degrees C or top liquid temperature above 80 degrees C would warrant further \\\
investigation and monitoring. If the transformer insulation is rated for 55 degrees C rise, a sustained winding operating temperature above 85 degrees C or \\\
top liquid temperature above 70 degrees C would warrant further investigation and monitoring. \\\ 
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'N2-High O2
The nitrogen gas blanket of this transformer was found to contain a high level of oxygen. Oxygen in the gas space can contribute to fluid \\\
deterioration and provide for a combustible atmosphere inside the transformer.  Recommend purging the gas space to less than 3% oxygen. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Paint
The paint condition of this equipment has deteriorated. Application of paint can protect against severe deterioration caused by \\\
destructive atmospheric elements. Because electrical equipment generate internal heat, this protective coating must permit heat to escape. \\\
Corroded surfaces do not allow proper heat transfer.  Recommend removal of any rust and application of a protective coat of paint. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\ 

'Pressurize Transformer
The transformer is not under a positive pressure.  Recommend adding necessary nitrogen to provide a positive pressure. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Repair Nitrogen Leak
The oxygen content in the nitrogen blanket is elevated.  Recommend performing investigation to locate leaks in the transformer gas \\\
blanket. An outage to replace gaskets may be necessary. Purge the nitrogen blanket to less than 3% to reduce oxidation of \\\
the fluid. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Repair P/V Gauge
The pressure/vacuum gauge is damaged. Replace gauge. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Sample Valve Seep
A seep of fluid from the sample valve was noted during this inspection. Valve should be monitored to determine if the seep continues. Repair valve \\\
if necessary. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Tank Grounds
A transformer tank ground was not visible during inspection. Recommend installing adequate tank ground. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Valve Leak-Monitor
HVA small leak at the sample valve was noted during this inspection. Sample valve should be monitored to determine if the leak continues. Re-packing of the existing \\\
valve or replacement may be necessary. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'Valve-Repair/Replace
Recommend repairing the leak noted at the sample valve. Re-packing of the existing valve or replacement is necessary. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'X0 Bushing
X0 BUSHING NOT GROUNDED ----- The X0 bushing on this transformer is not grounded. There are three major problems in operating an ungrounded system \\\
Transient overvoltages to ground may puncture the insulation.  Locating a ground fault may be difficult. Destructive arcing can occur if a second \\\
ground fault occurs before the first fault is cleared \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

:END

VIRTUAL FRONT PANEL_Comments: 

:END

VIRTUAL FRONT PANEL_Deficiencies:

:END

DefaultComments:


'INSTRUCTIONS:
'User to select best comment description from list below. On multi-line comments, user MUST \\\
'select the first line of the selection to include the entire selection.


'SEPARATOR LINE
'Use to separate manually entered statements.
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


'GENERAL COMMENTS

Based on the inspections, maintenance, calibration, and test results performed, this equipment was found to meet applicable NETA standards. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Some test results for this equipment are below applicable NETA standards.  Equipment with test results below NETA recommended values have an increased \\\
chance of in service failure. Based on historical test data for this equipment, the test results remain relatively constant. The customer should schedule \\\
this equipment for replacement or refurbishment. Recommend close monitoring of this equipment and increased test periods until the equipment is replaced \\\
or refurbished. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

INSULATION RESISTANCE:  As described in the Deficiencies and Recommendations Section, insulation resistance test results do not meet NETA minimum \\\
recommended values.  Normally, equipment with test values not meeting the minimum NETA recommendations should be considered for repair or replacement \\\
because it poses increased risk of in-service failure. However, after a review of historical test data for this equipment, it is apparent that the \\\
insulation resistance test results remain comparable with previous values. Based on the reduced but stable test results obtained and the current physical \\\
condition of this equipment, it is logical to conclude that this equipment poses only a slightly increased risk of in-service failure at this time. At the \\\
customers discretion, it would be reasonable for this equipment to remain in service if closely monitored through a program that includes annual inspections, \\\
testing, and maintenance. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

CONTACT RESISTANCE:  As described in the Deficiencies and Recommendations Section, the contact resistance test values do not strictly adhere to the NETA \\\
recommendations for maximum deviation of contact resistance between poles or similar devices. However, since all contact resistance test values were found to \\\
be significantly low in magnitude, this broader range of values is considered acceptable. Thermographic inspection during normal loading can detect problems \\\
associated with high contact resistance because high contact resistance results in abnormally high equipment or component temperatures. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


See Engineering Report for additional comments and recommendations. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

A more detailed course of corrective action can be presented after the next set of tests are performed and the results are evaluated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


Insulating members were found dirty and required cleaning. Insulating members were cleaned. Acceptable as left. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Equipment is dirty and requires cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Compartment heaters are not on. Compartment is subject to condensation. Repair heaters. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Compartment contains no heaters.  Compartment is subject to condensation.  Recommend installing necessary heaters to eliminate condensation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Equipment shows signs of physical damage.  Recommend repairing/replacing damaged components. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Switchboard, cables, bus and support insulators required cleaning.  Any dirt or foreign material that interfered with cooling or that reduced \\\
clearances to ground was removed. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


:END


'>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
'>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
'>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



DefaultDeficiencies:


'INSTRUCTIONS:
'User to select best deficiency description from list below. On multi-line deficiencies, user MUST \\\
'select the first line of the selection to include the entire selection.

'SEPARATOR LINE
'Use to separate manually entered statements.
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'GENERAL DEFICIENCIES
DANGER. DO NOT RETURN THIS EQUIPMENT TO SERVICE WITHOUT REPAIRS. This equipment failed the electrical test(s) performed. \\\
This equipment should not be returned to service. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Some test results for this equipment are below applicable NETA standards.  Equipment with test results below NETA recommended values have an increased \\\
chance of in service failure. Based on historical test data for this equipment, the test results remain relatively constant. The customer should schedule \\\
this equipment for replacement or refurbishment. Recommend close monitoring of this equipment and increased test periods until the equipment is replaced \\\
or refurbished. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

See Engineering Report for additional comments and recommendations. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

A more detailed course of corrective action can be presented after the next set of tests are performed and the results are evaluated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Insulating members are dirty and require cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Equipment was found not properly anchored. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Equipment was found without proper ground connections. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

See Engineering Report for additional comments and recommendations. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Equipment was found extremely dirty and requires extensive cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

Insulating members are dirty and require cleaning. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


'GENERAL TESTS

'CONTACT RESISTANCE
CONTACT RESISTANCE ----- The contact resistance test values deviate by more than 50%.  Recommend investigation. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

HIGH CONTACT RESISTANCE ----- Contact resistance test results are higher than other similar equipment.  Recommend investigation. \\\
Perform infrared inspection to determine if excessive heating is a problem. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

'INSULATION POWER FACTOR
HIGH INSULATION POWER FACTOR ----- Insulation power factor test results are rated deteriorated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

HIGH WATTS LOSS ----- Watts loss test results are rated deteriorated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


'INSULATION RESISTANCE
LOW PI ----- The polarization index ratio is rated deteriorated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

LOW DAR ----- The dielectric absorption ratio is rated deteriorated. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\

LOW INSULATION RESISTANCE ----- Insulation resistance test results are below the recommended value. \\\
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- \\\


:END



Condition:

Acceptable
Dirty
Deteriorated
Not Applicable
Not Inspected
New
Poor
See Comment
See Deficiency
:END


Lube:

Cleaned
Cleaned/Lubed
Lubed
Not Applicable
:END


Manufacturer:
ABB
ABB Power T&D Co
Aetna
Allen Bradley
Allis-Chalmers
Alstom T&D - Balteau
ASEA
Brown-Boveri
Challenger
Cooper Power Systems
Craig Electric
Cutler-Hammer
Delta Star, Inc
Eaton
Federal Pacific Electric
Feranti Packard
Furnas
General Cable
General Electric
GS - Hevi Duty
Hevi Duty Electric
Howard Industries, Inc
Line Material
McGraw Edison
No Nameplate
Okonite
Olsun
S&C Electric Company
Schneider Canada
Siemens
Siemens Energy & Automation
Solar
Sorgel-Square D
Square D
Sylvania
Topaz DPP Division
Unknown
Wagner Electric
Westinghouse
:END

AmpRating:
15
20
25
30
35
40
45
50
60
70
80
90
100
110
125
150
175
200
225
250
300
350
400
450
500
600
700
800
1000
1200
1600
2000
2500
3000
4000
5000
6000
:END

NomVolts:
120
120/240
208Y/120
240
347
480Y/277
480
600Y/347
600
:END

InstallMethod:

cable tray
duct bank
:END
      џџ  OPdbFormsEmbeddedWsts5;iE#      ?  ?                           Optima Data - Do Not Change tE#џџџ                               iF#	             џџ   OOptDrawBorderData                                џџџџ           џџџџ                 џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџџџ   OFormsToolEmbeddedWs76       5>              lE#  __Optima Data - Do Not Change            ts5;iE#      ?  ?                           Custom Field Entry tE#џџџ     P  (                       iF#	                                             џџџџ           џџџџ                 џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџ76       5>        P  (   lE#  __Custom Field Entry     ЭЭЭЭ   џўџD a t e M o d i f i e d        џa3ф@џўџD a t e C r e a t e d        ПMёЎт@џўџ	D a t e S y n c h        шДєДу@џўџb I s D e l         џўџ
D e l e t e I n f o    џўџ џўџ
b I s R e t i r e d        џўџb O p t i m a F o r m        џўџb S u g g e s t U p d a t e        џўџ	L a s t M o d B y    џўџ џўџC r e a t o r G u i d    џўџ џўџT e m p l a t e s U s e d    џўџ8O p t i m a   D a t a   -   D o   N o t   C h a n g e | C u s t o m   F i e l d   E n t r y | U s e r   D a t a џўџE x t e r n a l T e m p l a t e s    џўџ џўџT e m p l a t e F a m i l y    џўџ џўџT e m p l a t e M a n u    џўџ џўџb I s L o c a l F o r m         џўџS y n c h W r i t e D a t e        Gь;ЖТ+п@џўџS t a t F l g         џўџb L o c k e d        џўџL o c k I n f o    џўџ џўџxB I 5 O H H J S M K 5 2 U Y V M 7 H P 6 
 T e m p l a t e :   C u s t o m   F i e l d   E n t r y 
 D a t e   C r e a t e d :   ! 2 0 0 4 , 0 1 , 0 6 , 0 8 , 2 0 , 0 4 
 D a t e   M o d i f i e d :   ! 2 0 0 4 , 0 2 , 1 7 , 1 0 , 0 8 , 0 8    џўџT e m p l a t e G U I D    џўџB I 5 O H H J S M K 5 2 U Y V M 7 H P 6 џўџN a m e    џўџC u s t o m   F i e l d   E n t r y џўџ
D e f i n i t i o n    У  optform          SAVDOPTys65=?   A  >                =       ?       ?         џџ  OOptDrawCtrlScriptmE#On Initial Update     mE#	On Update      mE#On Page Set Focus     mE#On Form Exit     mE#On Timer 	    mE#Global Functions && Variables        џџџџџџџ 0                mE#         $2                џџџ P  (   (   P                mE#On Initial Update     mE#	On Update      mE#On Page Set Focus     mE#On Form Exit     mE#On Timer 	    mE#Global Functions && Variables       џџ  OPdbFormsEmbeddedWsts5;iE#NaМ   ?  ?                           Custom Field Entry tE#џџџ                                 iF#	             џџ   OOptDrawBorderData                                џџџџ           џџџџ                 џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџџџ   OFormsToolEmbeddedWs76       5>ЭЭЭЭЭЭЭЭЭЭЭЭЭЭЭЭlE#        ЭЭЭЭ   џўџD a t e M o d i f i e d        Ш[Mт@џўџD a t e C r e a t e d        ?овт@џўџ	D a t e S y n c h        ?овт@џўџb I s D e l         џўџ
D e l e t e I n f o    џўџ џўџ
b I s R e t i r e d        џўџb O p t i m a F o r m        џўџb S u g g e s t U p d a t e        џўџ	L a s t M o d B y    џўџ џўџC r e a t o r G u i d    џўџ џўџT e m p l a t e s U s e d    џўџ џўџE x t e r n a l T e m p l a t e s    џўџ џўџT e m p l a t e F a m i l y    џўџ џўџT e m p l a t e M a n u    џўџ џўџb I s L o c a l F o r m         џўџS y n c h W r i t e D a t e        Gь;ЖТ+п@џўџS t a t F l g         џўџb L o c k e d        џўџL o c k I n f o    џўџ џўџB X I R A A A I D P Q 6 H 3 W Z D M K N 
 T e m p l a t e :   N e w J o b S u b f o r m C o n f i r m J o b C r e a t e 
 D a t e   C r e a t e d :   ! 2 0 1 1 , 0 4 , 2 2 , 1 4 , 3 4 , 2 1 
 D a t e   M o d i f i e d :   ! 2 0 1 5 , 0 7 , 2 0 , 0 9 , 1 8 , 3 4    џўџT e m p l a t e G U I D    џўџB X I R A A A I D P Q 6 H 3 W Z D M K N џўџN a m e    џўџN e w J o b S u b f o r m C o n f i r m J o b C r e a t e џўџ
D e f i n i t i o n    4  optform          SAVDOUT}s65>?џўџ   А@  `@                =                         џџ  OOptDrawCtrlScriptmE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџ     mE#џўџD a t a џўџ            РРР џўџ0                џўџ             mE#         $2         џўџ       РРР p  0  0  p                mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџ     mE#џўџD a t a џўџ     џўџ  џџ  OPdbDrawText   tE#РРР     ^                        iF#	             џџ   OOptDrawBorderData                            џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџџџ   OPdbFormsToolText76        77    џџ   OPdbFormsTagInfo        џўџ            ў  Ѕ   lE#     џўџC u s t o m e r      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     з          Њ               iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ         Њ   w  Ш   lE#     џўџS i t e I D      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#џџџ     У   2     Ь        iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ       Ь  b  ў  lE#     S t a r t   I n s p e c t i o n џўџJ o b C r e a t e S t a r t     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџJC a l l   C r e a t e J o b A d d A s s e t ( )  
  
 C a l l   S e t v a r ( " t x t _ G e t S t a t e " ,   G e t v a r ( " S t a t e " ) )  
     mE#џўџO n   E n t e r џўџ      mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџS t a r t   I n s p e c t i o n     9у=                                     ЭV#              џўџA r i a l џџџ                tE#РРР     d      2      2           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     2         Ѕ   lE#    S i t e   O w n e r : џўџC u s t o m e r L b l      ЭЭЭЭ   qE  џўџS i t e   O w n e r :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     i      -   Њ   -           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     -   Њ      Ш   lE#    S i t e   I D : џўџS i t e I d L a b e l      ЭЭЭЭ   qE  џўџS i t e   I D :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР              Э              iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ        Э      ы   lE#    S i t e   A d d r e s s : џўџ      ЭЭЭЭ   qE  џўџS i t e   A d d r e s s :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     K      K     K           iF#	                                            џџџџ           џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     K        1  lE# 
   C i t y : џўџ             qE  џўџC i t y :     9у=                                     V#              џўџA r i a l џџџ                tE#РРР     }         6             iF#	                                            џџџџ           џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ        6     T  lE#    P o s t a l   C o d e : џўџ             qE  џўџP o s t a l   C o d e :     9у=                                     V#              џўџA r i a l џџџ                tE#РРР     P      F   Y  F           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     F   Y     w  lE# 
   T a s k : џўџ      ЭЭЭЭ   qE  џўџT a s k :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     Z      <   |  <           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     <   |       lE#    I n s t a n c e : џўџ      ЭЭЭЭ   qE  џўџ	I n s t a n c e :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР              Э               iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ         Э   4  ы   lE#     џўџA d d r e s s      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     з          |              iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ         |  w    lE#     џўџT a g I D      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР              №               iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ         №   4    lE#     џўџA d d r e s s 2      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     Џ      |    |          iF#	                                            џџџџ    	       џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     |    +  1  lE# "   S t a t e   /   P r o v i n c e : џўџ             qE  џўџS t a t e   /   P r o v i n c e :     9у=                                     V#              џўџA r i a l џџџ                tE#РРР     s      И  6  И          iF#	                                            џџџџ    
       џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     И  6  +  T  lE#    C o u n t r y : џўџ             qE  џўџC o u n t r y :     9у=                                     V#              џўџA r i a l џџџ                tE#РРР     з                        iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ           w  1  lE#     џўџC i t y      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     з          6              iF#	                                         џџџџ   
       џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77           џўџ         6  w  T  lE#     џўџZ i p     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   I n i t i a l   U p d a t e џўџ(' p d b l i t e _ n o s c r i p t  
  
 ' . t h i s ! r e a d o n l y   =   1     mE#џўџO n   R i g h t   C l i c k џўџ 
           qE  џўџ     9у=                                        V#              џўџA r i a l џџџ                tE#РРР     њ      :  6  :          iF#	                                         џџџџ          џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77           џўџ     :  6  4  T  lE#     џўџC o u n t r y     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   I n i t i a l   U p d a t e џўџ 
     mE#џўџO n   R i g h t   C l i c k џўџ 
           qE  џўџ     9у=                                        V#              џўџA r i a l џџџ                tE#РРР       #   -   -   -           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     -   -   /  P   lE# J   C O N F I R M   I N S T A N C E   A N D   S I T E   I N F O R M A T I O N џўџC o n f i r m J o b C r e a t e H e a d i n g      ЭЭЭЭ   qE  џўџ%C O N F I R M   I N S T A N C E   A N D   S I T E   I N F O R M A T I O N     ЋЊ*>                                      ЭV#             џўџA r i a l џџџ                tE#џџџ     n   2   џ   Ь  џ       iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     џ   Ь  m  ў  lE#    B a c k џўџJ o b C r e a t e B a c k     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџc a l l   G o B a c k ( )     mE#џўџO n   E n t e r џўџ      mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџB a c k     9у=                                     ЭV#              џўџA r i a l џџџ                tE#РРР     з          Y              iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ         Y  w  w  lE#     џўџT i c k e t      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     }      Ў  Y  Ў          iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Ў  Y  +  w  lE#    A t t e n t i o n : џўџ      ЭЭЭЭ   qE  џўџ
A t t e n t i o n :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     њ      :  Y  :          iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     :  Y  4  w  lE#     џўџC u s t o m e r A t t n      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     O                    iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ          W  Ѕ   lE#     џўџC u s t o m e r E r r     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#џ            џўџA r i a l џџџ                tE#РРР     њ      :  6  :          iF#	                                         џџџџ   	       џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77           џўџ     :  6  4  T  lE#     џўџS t a t e     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
           qE  џўџ     9у=                                        V#              џўџA r i a l џџџ                tE#РРР     њ      :    :          iF#	                                         џџџџ   	       џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77           џўџ     :    4  1  lE#     џўџS t a t e _ i n p u t     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
           qE  џўџ     9у=                                        V#              џўџA r i a l џџџ             џўџD a t e M o d i f i e d        ЋiЌф@џўџD a t e C r e a t e d        сznѓйу@џўџ	D a t e S y n c h        сznѓйу@џўџb I s D e l         џўџ
D e l e t e I n f o    џўџ џўџ
b I s R e t i r e d        џўџb O p t i m a F o r m        џўџb S u g g e s t U p d a t e        џўџ	L a s t M o d B y    џўџ џўџC r e a t o r G u i d    џўџ џўџT e m p l a t e s U s e d    џўџ8U s e r   D a t a | O p t i m a   D a t a   -   D o   N o t   C h a n g e | C u s t o m   F i e l d   E n t r y џўџE x t e r n a l T e m p l a t e s    џўџ џўџT e m p l a t e F a m i l y    џўџ џўџT e m p l a t e M a n u    џўџ џўџb I s L o c a l F o r m         џўџS y n c h W r i t e D a t e        Gь;ЖТ+п@џўџS t a t F l g         џўџb L o c k e d        џўџL o c k I n f o    џўџ џўџ}B X I R A A A K 5 P 6 K A M B S F Q C N 
 T e m p l a t e :   N e w J o b S u b f o r m L o o k u p S i t e 
 D a t e   C r e a t e d :   ! 2 0 1 1 , 0 3 , 1 5 , 1 5 , 0 8 , 1 8 
 D a t e   M o d i f i e d :   ! 2 0 1 6 , 0 3 , 0 7 , 1 0 , 4 4 , 3 6    џўџT e m p l a t e G U I D    џўџB X I R A A A K 5 P 6 K A M B S F Q C N џўџN a m e    џўџN e w J o b S u b f o r m L o o k u p S i t e џўџ
D e f i n i t i o n    Ё  optform          SAVDOUT}s65>?џўџ   А@  `@                =                         џџ  OOptDrawCtrlScriptmE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ 
      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџ     mE#џўџD a t a џўџ            РРР џўџ0                џўџ             mE#         $2         џўџ       РРР p  0  0  p                mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ 
      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџ     mE#џўџD a t a џўџ     џўџ  џџ  OPdbDrawText   tE#РРР       #   d      d           iF#	             џџ   OOptDrawBorderData                                џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџџџ   OPdbFormsToolText76        77    џџ   OPdbFormsTagInfo        џўџ     d      u  7   lE#     џўџM a i n E r r     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ 
      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ       Р=                                       ЭV#             џўџA r i a l џџџ                tE#РРР     з      ;  №   ;          iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     ;  №       lE#     џўџT a g I D      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     }      Д   №   Д           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Д   №   1    lE#    I n s t a n c e : џўџ      ЭЭЭЭ   qE  џўџ	I n s t a n c e :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     }      Д     Д           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Д     1  6  lE#    S i t e   I D : џўџ      ЭЭЭЭ   qE  џўџS i t e   I D :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     }      Д   Ш   Д           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Д   Ш   1  ц   lE# 
   T a s k : џўџ      ЭЭЭЭ   qE  џўџT a s k :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#џџџ     n   2     ж        iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ       ж      lE#    B a c k џўџL o o k u p S i t e B a c k     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџc a l l   G o B a c k ( )     mE#џўџO n   E n t e r џўџ      mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџB a c k     9у=                                     ЭV#              џўџA r i a l џџџ                tE#џџџ     з      ;    ;          iF#	                                       џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     ;      6  lE#     џўџS i t e I D       ЭЭЭЭ   qE  џўџ    9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     з      ;  Ш   ;          iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     ;  Ш     ц   lE#     џўџT i c k e t      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     щ  #   K   2   K           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     K   2   4  U   lE#    L O O K U P   S I T E џўџL o o k u p S i t e H e a d i n g      ЭЭЭЭ   qE  џўџL O O K U P   S I T E     ЋЊ*>                                      ЭV#             џўџA r i a l џџџ                tE#РРР     O                   iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ         k  6  lE#     џўџ	S i t e I D E r r      ЭЭЭЭ   qE  џўџ     rЧ=                                       ЭV#џ            џўџA r i a l џџџ                tE#џџџ        2   Є  ж  Є      iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     Є  ж  :    lE#    L o o k u p   S i t e џўџL o o k u p S i t e B u t t o n     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџC a l l   L o o k u p S i t e ( )     mE#џўџO n   E n t e r џўџ      mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџL o o k u p   S i t e     9у=                                     ЭV#              џўџA r i a l џџџ                tE#РРР     K      %  #   %          iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџ76        77            џўџ     %  #   p  7   lE#  џўџ      ЭЭЭЭ   qE  џўџ       Р=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     K                       iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџ76        77            џўџ           _   (   lE#  џўџ      ЭЭЭЭ   qE  џўџ       Р=                                       ЭV#              џўџA r i a l џџџ                tE#џџџ     с   2   6  @  6      iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     6  @    r  lE# .   L o o k u p   S i t e   B y   C u s t o m e r џўџL o o k u p S i t e B y C u s t o m e r B u t t o n     mE#џўџO n   I n i t i a l   U p d a t e џўџ. t h i s ! v i s i b l e   =   f a l s e     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ9C a l l   S e t V a r ( " S i t e I D E r r " ,   " " )  
 C a l l   L o o k u p S i t e B y C u s t o m e r ( )     mE#џўџO n   E n t e r џўџ      mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџL o o k u p   S i t e   B y   C u s t o m e r     9у=                                     ЭV#              џўџA r i a l џџџ             џўџD a t e M o d i f i e d        ЪSИф@џўџD a t e C r e a t e d        IР7/4еу@џўџ	D a t e S y n c h        IР7/4еу@џўџb I s D e l         џўџ
D e l e t e I n f o    џўџ џўџ
b I s R e t i r e d        џўџb O p t i m a F o r m        џўџb S u g g e s t U p d a t e        џўџ	L a s t M o d B y    џўџ џўџC r e a t o r G u i d    џўџ џўџT e m p l a t e s U s e d    џўџ8U s e r   D a t a | O p t i m a   D a t a   -   D o   N o t   C h a n g e | C u s t o m   F i e l d   E n t r y џўџE x t e r n a l T e m p l a t e s    џўџ џўџT e m p l a t e F a m i l y    џўџ џўџT e m p l a t e M a n u    џўџ џўџb I s L o c a l F o r m         џўџS y n c h W r i t e D a t e        Gь;ЖТ+п@џўџS t a t F l g         џўџb L o c k e d        џўџL o c k I n f o    џўџ џўџB X I R A A A L 5 P 7 W F A G S F Q C N 
 T e m p l a t e :   N e w J o b S u b f o r m C o n f i r m S i t e C r e a t e 
 D a t e   C r e a t e d :   ! 2 0 1 1 , 0 3 , 3 1 , 2 2 , 5 7 , 4 8 
 D a t e   M o d i f i e d :   ! 2 0 1 4 , 1 1 , 1 9 , 1 0 , 1 3 , 5 7    џўџT e m p l a t e G U I D    џўџB X I R A A A L 5 P 7 W F A G S F Q C N џўџN a m e    џўџN e w J o b S u b f o r m C o n f i r m S i t e C r e a t e џўџ
D e f i n i t i o n    я  optform          SAVDOUT{s65>?   А@  `@                =                         џџ  OOptDrawCtrlScriptmE#On Initial Update     mE#	On UpdateџBUIState = GetVar("UIState")

If UIState = "ConfirmSiteCreate" Then
    
    .CreateSiteMsg1 = "The Site ID " & GetVar("SiteID") & " does not exist. "
    .CreateSiteMsg2 = "Click [CREATE NEW SITE] to create a new Site in PowerDB."
    .CreateSiteMsg3 = "Click [BACK] to modify the Site ID and try again."

End If
     mE#On Page Set Focus     mE#On Form Exit     mE#On Timer 	    mE#Global Functions && Variables     mE#Data            РРР 0                            mE#         $2                РРР p  0  0  p                mE#On Initial Update     mE#	On UpdateџBUIState = GetVar("UIState")

If UIState = "ConfirmSiteCreate" Then
    
    .CreateSiteMsg1 = "The Site ID " & GetVar("SiteID") & " does not exist. "
    .CreateSiteMsg2 = "Click [CREATE NEW SITE] to create a new Site in PowerDB."
    .CreateSiteMsg3 = "Click [BACK] to modify the Site ID and try again."

End If
     mE#On Page Set Focus     mE#On Form Exit     mE#On Timer 	    mE#Global Functions && Variables     mE#Data       џџ  OPdbDrawText   tE#РРР     з      |     |          iF#	             џџ   OOptDrawBorderData                            џџџџ            џџџџ                 џџџџџџџџ  џ         џџџџџџџџџџџџџџ   OPdbFormsToolText76        77    џџ   OPdbFormsTagInfo             |     S  Ѕ   lE#     Ticket     ЭЭЭЭ   qE       9у=                                       ЭV#              Arialџџџ                tE#РРР     X                     iF#	                                             џџџџ            џџџџ                 џџџџџџџџ  џ         џџџџџџџџџџџџ76        77                      ю  "  lE#     CreateSiteMsg1    mE#On Initial Update     mE#	On Update      mE#	On Select     mE#On Enter     mE#On Right Click 
    ЭЭЭЭ   qE       9у=                                       ЭV#              Arialџџџ                tE#РРР     }      ѕ   Э   ѕ           iF#	                                             џџџџ            џџџџ                 џџџџџџџџ  џ        џџџџџџџџџџџџ76        77                 ѕ   Э   r  ы   lE#    S i t e   I D : SiteIdLabel     ЭЭЭЭ   qE  Site ID:    9у=                                      ЭV#              Arialџџџ                tE#РРР     }      ѕ      ѕ           iF#	                                             џџџџ            џџџџ                 џџџџџџџџ  џ         џџџџџџџџџџџџ76        77                 ѕ      r  Ѕ   lE# 
   T a s k :       ЭЭЭЭ   qE  Task:    9у=                                      ЭV#              Arialџџџ                tE#РРР     X        @             iF#	                                             џџџџ            џџџџ                 џџџџџџџџ  џ         џџџџџџџџџџџџ76        77                    @  ю  ^  lE#     CreateSiteMsg3    mE#On Initial Update     mE#	On Update      mE#	On Select     mE#On Enter     mE#On Right Click 
    ЭЭЭЭ   qE       9у=                                       ЭV#              Arialџџџ                tE#РРР     }      ѕ   Њ   ѕ           iF#	                                             џџџџ            џџџџ                 џџџџџџџџ  џ         џџџџџџџџџџџџ76        77                 ѕ   Њ   r  Ш   lE#    I n s t a n c e :       ЭЭЭЭ   qE  	Instance:    9у=                                      ЭV#              Arialџџџ                tE#РРР     з      |  Э   |          iF#	                                         џџџџ            џџџџ                 џџџџџџџџ  џ         џџџџџџџџџџџџ76        77                 |  Э   S  ы   lE#     SiteID     ЭЭЭЭ   qE       9у=                                       ЭV#              Arialџџџ                tE#РРР     X        "             iF#	                                             џџџџ            џџџџ                 џџџџџџџџ  џ         џџџџџџџџџџџџ76        77                    "  ю  @  lE#     CreateSiteMsg2    mE#On Initial Update     mE#	On Update      mE#	On Select     mE#On Enter     mE#On Right Click 
    ЭЭЭЭ   qE       9у=                                       ЭV#              Arialџџџ                tE#РРР       #   Z   2   Z           iF#	                                             џџџџ            џџџџ                 џџџџџџџџ  џ        џџџџџџџџџџџџ76        77                 Z   2   ј  U   lE#    S I T E   N O T   F O U N D ConfirmSiteCreateHeading     ЭЭЭЭ   qE  SITE NOT FOUND    ЋЊ*>                                      ЭV#             Arialџџџ                tE#џџџ     s   2     ж        iF#	  АA                         џџџџ            џџџџ                 џџџџџџџџ  џ        џџџџџџџџџџџџ76        77                   ж  w    lE#    B a c k SiteCreateBack    mE#On Initial Update     mE#	On Update      mE#	On Selectcall GoBack()    mE#On Enter      mE#On Right Click 
    ЭЭЭЭ   qE  Back    9у=                                     ЭV#              Arialџџџ                tE#џџџ     з   2   Љ  ж  Љ      iF#	  АA                         џџџџ            џџџџ                 џџџџџџџџ  џ        џџџџџџџџџџџџ76        77                 Љ  ж      lE#    C r e a t e   N e w   S i t e SiteCreateNew    mE#On Initial Update     mE#	On Update      mE#	On Select6call ClearAddressFields()
call SetScreen("CreateNew")    mE#On Enter      mE#On Right Click 
    ЭЭЭЭ   qE  Create New Site    9у=                                     ЭV#              Arialџџџ                tE#РРР     з      |  Њ   |          iF#	                                         џџџџ            џџџџ                 џџџџџџџџ  џ        џџџџџџџџџџџџ76        77                 |  Њ   S  Ш   lE#     TagID     ЭЭЭЭ   qE       9у=                                       ЭV#              Arialџџџ             џўџD a t e M o d i f i e d        K~БЄM}ф@џўџD a t e C r e a t e d        8Џ&>зу@џўџ	D a t e S y n c h        8Џ&>зу@џўџb I s D e l         џўџ
D e l e t e I n f o    џўџ џўџ
b I s R e t i r e d        џўџb O p t i m a F o r m        џўџb S u g g e s t U p d a t e        џўџ	L a s t M o d B y    џўџ џўџC r e a t o r G u i d    џўџ џўџT e m p l a t e s U s e d    џўџ8U s e r   D a t a | O p t i m a   D a t a   -   D o   N o t   C h a n g e | C u s t o m   F i e l d   E n t r y џўџE x t e r n a l T e m p l a t e s    џўџ џўџT e m p l a t e F a m i l y    џўџ џўџT e m p l a t e M a n u    џўџ џўџb I s L o c a l F o r m         џўџS y n c h W r i t e D a t e        Gь;ЖТ+п@џўџS t a t F l g         џўџb L o c k e d        џўџL o c k I n f o    џўџ џўџyB X I R A A A L F P 2 C C E D S F Q C N 
 T e m p l a t e :   N e w J o b S u b f o r m L o o k u p 
 D a t e   C r e a t e d :   ! 2 0 1 1 , 0 3 , 1 5 , 2 2 , 5 0 , 0 4 
 D a t e   M o d i f i e d :   ! 2 0 1 6 , 0 4 , 0 4 , 0 9 , 0 4 , 1 9    џўџT e m p l a t e G U I D    џўџB X I R A A A L F P 2 C C E D S F Q C N џўџN a m e    џўџN e w J o b S u b f o r m L o o k u p џўџ
D e f i n i t i o n    E$  optform          SAVDOUT}s65>?џўџ   А@  `@                =                         џџ  OOptDrawCtrlScriptmE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџ     mE#џўџD a t a џўџ            РРР џўџ0                џўџ             mE#         $2         џўџ       РРР p  0  0  p                mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџ     mE#џўџD a t a џўџ     џўџ  џџ  OPdbDrawText   tE#РРР     K      %  #   %          iF#	             џџ   OOptDrawBorderData                                џџџџ           џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџџџ   OPdbFormsToolText76        77    џџ   OPdbFormsTagInfo        џўџ     %  #   p  7   lE#  џўџ      ЭЭЭЭ   qE  џўџ       Р=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     K                         iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџ76        77            џўџ            K   (   lE#  џўџ      ЭЭЭЭ   qE  џўџ       Р=                                       ЭV#              џўџA r i a l џџџ                tE#РРР       #   P      P           iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     P      a  7   lE#     џўџM a i n E r r     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ 
      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ       Р=                                       ЭV#             џўџA r i a l џџџ                tE#џџџ     з      1  О   1          iF#	                                       џџџџ          џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     1  О     м   lE#     џўџT i c k e t       ЭЭЭЭ   qE  џўџ    9у=                                       ЭV#              џўџA r i a l џџџ                tE#џџџ        2   c  Н  c      iF#	  АA                         џџџџ          џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     c  Н  є  я  lE#    L o o k u p џўџ	L o o k u p B t n     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџJi f   V a l i d a t e R e p o r t T y p e F i e l d   t h e n  
         C a l l   L o o k u p T i c k e t A n d T a g ( )  
 e n d   i f  
  
     mE#џўџO n   E n t e r џўџ      mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџL o o k u p     9у=                                     ЭV#              џўџA r i a l џџџ                tE#РРР     а  #   P   -   P           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     P   -      P   lE# 0   L O O K U P   I N S T A N C E   A N D   T A S K џўџL o o k u p H e a d i n g      ЭЭЭЭ   qE  џўџL O O K U P   I N S T A N C E   A N D   T A S K     ЋЊ*>                                      ЭV#             џўџA r i a l џџџ                tE#џџџ     з      1  ы   1          iF#	                                       џџџџ          џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     1  ы     	  lE#     џўџT a g I D       ЭЭЭЭ   qE  џўџ    9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     }      Њ   О   Њ           iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Њ   О   '  м   lE# 
   T a s k : џўџ      ЭЭЭЭ   qE  џўџT a s k :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     }      Њ   ы   Њ           iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Њ   ы   '  	  lE#    I n s t a n c e : џўџ      ЭЭЭЭ   qE  џўџ	I n s t a n c e :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     }      Њ     Њ           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Њ     '  6  lE#    R e p o r t   T y p e : џўџ     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџR e p o r t   T y p e :     9у=                                      ЭV#              џўџA r i a l џџџ             џџ  OptDrawDropDowntE#џџџ     з      1    1          iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџџџ   OPdbFormsToolDropDowne76
                       џўџ     1      6  lE#     џўџ
R e p o r t T y p e      mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ     9у=                                        ЭV#              џўџA r i a l џџџ                       ЭЭЭЭџўџ}1 0 7 7 0   B A T T E R Y  
 2 0 1 0 0   S I T E S C A N  
 8 0 2 0 0   A P M  
 8 0 1 0 0   A P S  
 8 0 3 0 0   N X   2 2 5 - 6 0 0  
 1 0 7 7 1   B A T T E R Y  
 T H E R M A L   P M V  
 8 0 8 0 0   U P S  
 E H S   S i t e   S u r v e y     џўџ       tE#џџџ        2   c  h  c      iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     c  h  є    lE#    U p d a t e   I n s t a n c e џўџU p d a t e T a g b t n     mE#џўџO n   I n i t i a l   U p d a t e џўџ. t h i s ! v i s i b l e   =   0     mE#џўџ	O n   U p d a t e џўџ#. t h i s ! v i s i b l e   =   G e t V a r ( " U p d a t e T a g " )      mE#џўџ	O n   S e l e c t џўџ%c a l l   C h a n g e F o r m i n T a g ( G e t V a r ( " T a g I D " ) )     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџU p d a t e   I n s t a n c e     9у=                                     ЭV#              џўџA r i a l џџџ                tE#РРР     T       О             iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ       О   f  м   lE#     џўџ	T i c k e t E r r     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ 
      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ     rЧ=                                       ЭV#џ            џўџA r i a l џџџ                tE#РРР     T       ы             iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ       ы   f  	  lE#     џўџT a g I D E r r      ЭЭЭЭ   qE  џўџ     rЧ=                                       ЭV#џ            џўџA r i a l џџџ                tE#РРР     Y                   iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ         f  6  lE#     џўџR e p o r t T y p e E r r     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ     rЧ=                                       ЭV#џ            џўџA r i a l џџџ             џўџD a t e M o d i f i e d        ћМф@џўџD a t e C r e a t e d        3(r>еу@џўџ	D a t e S y n c h        3(r>еу@џўџb I s D e l         џўџ
D e l e t e I n f o    џўџ џўџ
b I s R e t i r e d        џўџb O p t i m a F o r m        џўџb S u g g e s t U p d a t e        џўџ	L a s t M o d B y    џўџ џўџC r e a t o r G u i d    џўџ џўџT e m p l a t e s U s e d    џўџ8U s e r   D a t a | O p t i m a   D a t a   -   D o   N o t   C h a n g e | C u s t o m   F i e l d   E n t r y џўџE x t e r n a l T e m p l a t e s    џўџ џўџT e m p l a t e F a m i l y    џўџ џўџT e m p l a t e M a n u    џўџ џўџb I s L o c a l F o r m         џўџS y n c h W r i t e D a t e        Gь;ЖТ+п@џўџS t a t F l g         џўџb L o c k e d        џўџL o c k I n f o    џўџ џўџ|B X I R A A A L N P 4 L D E E S F Q C N 
 T e m p l a t e :   N e w J o b S u b f o r m C r e a t e N e w 
 D a t e   C r e a t e d :   ! 2 0 1 1 , 0 3 , 3 1 , 2 1 , 2 7 , 4 1 
 D a t e   M o d i f i e d :   ! 2 0 1 5 , 0 7 , 2 0 , 0 9 , 5 3 , 1 4    џўџT e m p l a t e G U I D    џўџB X I R A A A L N P 4 L D E E S F Q C N џўџN a m e    џўџN e w J o b S u b f o r m C r e a t e N e w џўџ
D e f i n i t i o n    Q9  optform          SAVDOUT}s65>?џўџ   А@  `@                =                         џџ  OOptDrawCtrlScriptmE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџ     mE#џўџD a t a џўџ            РРР џўџ0                џўџ             mE#         $2         џўџ       РРР p  0  0  p                mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџ     mE#џўџD a t a џўџ     џўџ  џџ  OPdbDrawText   tE#џџџ     ^     Д   i   Д           iF#	           џџ   OOptDrawBorderData                            џџџџ          џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџџџ   OPdbFormsToolText76        77    џџ   OPdbFormsTagInfo        џўџ     Д   i        lE#     џўџC u s t o m e r       ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#џџџ          Д   "  Д           iF#	                                       џџџџ          џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Д   "  H  @  lE#     џўџA d d r e s s 2       ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#џџџ          Д   њ   Д           iF#	                                       џџџџ          џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Д   њ   H    lE#     џўџA d d r e s s       ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     Д        J            iF#	                                            џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ       J  D  h  lE# "   S t a t e   /   P r o v i n c e : џўџ             qE  џўџS t a t e   /   P r o v i n c e :     9у=                                     V#              џўџA r i a l џџџ                tE#РРР           Ў  r  Ў          iF#	                                            џџџџ           џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     Ў  r  D    lE#    C o u n t r y : џўџ             qE  џўџC o u n t r y :     9у=                                     V#              џўџA r i a l џџџ                tE#џџџ     в      Д   J  Д           iF#	                                       џџџџ          џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Д   J    h  lE#     џўџC i t y       ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#џџџ     в      Д   r  Д           iF#	                                       џџџџ          џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77           џўџ     Д   r      lE#     џўџZ i p      mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   I n i t i a l   U p d a t e џўџ(' p d b l i t e _ n o s c r i p t  
  
 ' . t h i s ! r e a d o n l y   =   1     mE#џўџO n   R i g h t   C l i c k џўџ 
           qE  џўџ     9у=                                        V#              џўџA r i a l џџџ                tE#џџџ     њ      N  J  N          iF#	                                       џџџџ          џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77           џўџ     N  J  H  h  lE#     џўџS t a t e      mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
           qE  џўџ     9у=                                        V#              џўџA r i a l џџџ                tE#џџџ     њ      N  r  N          iF#	                                       џџџџ          џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77           џўџ     N  r  H    lE#     џўџC o u n t r y      mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   I n i t i a l   U p d a t e џўџ 
     mE#џўџO n   R i g h t   C l i c k џўџ 
           qE  џўџ     9у=                                        V#              џўџA r i a l џџџ                tE#РРР     n      <   i   <           iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     <   i   Њ      lE#    S i t e   O w n e r : џўџC u s t o m e r L b l      ЭЭЭЭ   qE  џўџS i t e   O w n e r :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     _      K      K           iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     K      Њ   Џ   lE#    S i t e   I D : џўџS i t e I d L a b e l      ЭЭЭЭ   qE  џўџS i t e   I D :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     _      K   Д   K           iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     K   Д   Њ   в   lE# 
   T a s k : џўџ      ЭЭЭЭ   qE  џўџT a s k :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     d      F   з   F           iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     F   з   Њ   ѕ   lE#    I n s t a n c e : џўџ      ЭЭЭЭ   qE  џўџ	I n s t a n c e :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР           (   њ   (           iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     (   њ   Њ     lE#    S i t e   A d d r e s s : џўџ      ЭЭЭЭ   qE  џўџS i t e   A d d r e s s :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     F      d   J  d           iF#	                                            џџџџ           џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     d   J  Њ   h  lE# 
   C i t y : џўџ             qE  џўџC i t y :     9у=                                     V#              џўџA r i a l џџџ                tE#РРР           (   r  (           iF#	                                            џџџџ           џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     (   r  Њ     lE#    P o s t a l   C o d e : џўџ             qE  џўџP o s t a l   C o d e :     9у=                                     V#              џўџA r i a l џџџ                tE#РРР     з      Д      Д           iF#	                                         џџџџ    	       џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Д        Џ   lE#     џўџS i t e I D      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     З  #   i   -   i           iF#	                                             џџџџ    
       џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     i   -      P   lE# ,   E N T E R   S I T E   I N F O R M A T I O N џўџC r e a t e N e w H e a d i n g      ЭЭЭЭ   qE  џўџE N T E R   S I T E   I N F O R M A T I O N     ЋЊ*>                                      ЭV#             џўџA r i a l џџџ                tE#џџџ     Э   2   Є  х  Є      iF#	  АA                         џџџџ          џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Є  х  q    lE#     S t a r t   I n s p e c t i o n џўџC r e a t e N e w S t a r t     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџMc a l l   C r e a t e J o b C r e a t e A s s e t ( )  
  
 C a l l   S e t v a r ( " t x t _ G e t S t a t e " ,   G e t v a r ( " S t a t e " ) )  
     mE#џўџO n   E n t e r џўџ      mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџS t a r t   I n s p e c t i o n     9у=                                     ЭV#              џўџA r i a l џџџ                tE#РРР     з      Д   Д   Д           iF#	                                         џџџџ           џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Д   Д     в   lE#     џўџT i c k e t      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#џџџ     У                    iF#	                                       џџџџ
   	       џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77           џўџ         S  И  lE#     џўџL a s t N a m e      mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   I n i t i a l   U p d a t e џўџ(' p d b l i t e _ n o s c r i p t  
  
 ' . t h i s ! r e a d o n l y   =   1     mE#џўџO n   R i g h t   C l i c k џўџ 
           qE  џўџ     9у=                                        V#              џўџA r i a l џџџ                tE#РРР     з      Д   з   Д           iF#	                                         џџџџ           џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     Д   з     ѕ   lE#     џўџT a g I D      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#џџџ     n   2   џ   х  џ       iF#	  АA                         џџџџ   
       џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     џ   х  m    lE#    B a c k џўџC r e a t e N e w B a c k     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ-c a l l   S e t V a r ( " C u s t o m e r E r r " ,   " " )  
 c a l l   G o B a c k ( )     mE#џўџO n   E n t e r џўџ      mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџB a c k     9у=                                     ЭV#              џўџA r i a l џџџ                tE#џџџ     в      Д     Д           iF#	                                       џџџџ	          џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77           џўџ     Д       И  lE#     џўџ	F i r s t N a m e      mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   I n i t i a l   U p d a t e џўџ(' p d b l i t e _ n o s c r i p t  
  
 ' . t h i s ! r e a d o n l y   =   1     mE#џўџO n   R i g h t   C l i c k џўџ 
           qE  џўџ     9у=                                        V#              џўџA r i a l џџџ                tE#РРР           (     (           iF#	                                            џџџџ           џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     (     Њ   И  lE#    A t t e n t i o n : џўџ             qE  џўџ
A t t e n t i o n :     9у=                                     V#              џўџA r i a l џџџ                tE#РРР     d      ц   Н  ц           iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     ц   Н  J  б  lE#    ( F i r s t   N a m e ) џўџ      ЭЭЭЭ   qE  џўџ( F i r s t   N a m e )       Р=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     d      Н  Н  Н          iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     Н  Н  !  б  lE#    ( L a s t   N a m e ) џўџ      ЭЭЭЭ   qE  џўџ( L a s t   N a m e )       Р=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     O       i             iF#	                                             џџџџ           џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ       i   k     lE#     џўџC u s t o m e r E r r     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#џ            џўџA r i a l џџџ             џўџD a t e M o d i f i e d        &Љж.­ф@џўџD a t e C r e a t e d        ЛЦ|<зу@џўџ	D a t e S y n c h        ЛЦ|<зу@џўџb I s D e l         џўџ
D e l e t e I n f o    џўџ џўџ
b I s R e t i r e d        џўџb O p t i m a F o r m        џўџb S u g g e s t U p d a t e        џўџ	L a s t M o d B y    џўџ џўџC r e a t o r G u i d    џўџ џўџT e m p l a t e s U s e d    џўџ8U s e r   D a t a | O p t i m a   D a t a   -   D o   N o t   C h a n g e | C u s t o m   F i e l d   E n t r y џўџE x t e r n a l T e m p l a t e s    џўџ џўџT e m p l a t e F a m i l y    џўџ џўџT e m p l a t e M a n u    џўџ џўџb I s L o c a l F o r m         џўџS y n c h W r i t e D a t e        Gь;ЖТ+п@џўџS t a t F l g         џўџb L o c k e d        џўџL o c k I n f o    џўџ џўџB X I R A A A L V P 4 H E E F S F Q C N 
 T e m p l a t e :   N e w J o b S u b f o r m C o n f i r m T a g C r e a t e 
 D a t e   C r e a t e d :   ! 2 0 1 1 , 0 3 , 3 1 , 2 2 , 4 5 , 5 9 
 D a t e   M o d i f i e d :   ! 2 0 1 6 , 0 4 , 0 4 , 0 9 , 0 4 , 0 3    џўџT e m p l a t e G U I D    џўџB X I R A A A L V P 4 H E E F S F Q C N џўџN a m e    џўџN e w J o b S u b f o r m C o n f i r m T a g C r e a t e џўџ
D e f i n i t i o n    Б2  optform          SAVDOUT}s65>?џўџ   А@  `@                =                         џџ  OOptDrawCtrlScriptmE#џўџO n   I n i t i a l   U p d a t e џўџџmc a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( . F o r m I d , " s e l e c t     R e g i o n G U I D ,   R e g i o n D e s c r i p   f r o m   P d b R e g i o n s   w h e r e   b I s D e l   =   0   a n d   b I s A c t i v e   =   1   o r d e r   b y   R e g i o n D e s c r i p   " , t r u e )  
  
 N u m F o r m s   =   G e t V a r ( " d b _ N u m R e c o r d s " )  
  
 L i s t   =   " "  
 f o r   i   =   1   t o   N u m F o r m s  
         D e v i c e N a m e   =   G e t N d x V a r ( " d b _ R e g i o n D e s c r i p " , i )  
           L i s t   =   L i s t   +   C S t r ( D e v i c e N a m e )   +   " \ n "  
 n e x t  
  
 c a l l   S e t V a r ( " R e g i o n L i s t " , L i s t )  
     mE#џўџ	O n   U p d a t e џўџџMU I S t a t e   =   G e t V a r ( " U I S t a t e " )  
  
 I f   U I S t a t e   =   " C o n f i r m T a g C r e a t e "   T h e n  
          
         . C r e a t e T a g M s g 1   =   " T h e   I n s t a n c e   "   &   G e t V a r ( " T a g I D " )   &   "   d o e s   n o t   e x i s t .   "  
         . C r e a t e T a g M s g 2   =   " C l i c k   [ C r e a t e   N e w   I n s t a n c e ]   t o   c r e a t e   a   n e w   i n s t a n c e   i n   P o w e r D B . "  
         . C r e a t e T a g M s g 3   =   " C l i c k   [ B a c k ]   t o   m o d i f y   t h e   I n s t a n c e   a n d   t r y   a g a i n . "  
          
  
 E n d   I f  
      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџ     mE#џўџD a t a џўџ            РРР џўџ0                џўџ             mE#         $2         џўџ       РРР p  0  0  p                mE#џўџO n   I n i t i a l   U p d a t e џўџџmc a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( . F o r m I d , " s e l e c t     R e g i o n G U I D ,   R e g i o n D e s c r i p   f r o m   P d b R e g i o n s   w h e r e   b I s D e l   =   0   a n d   b I s A c t i v e   =   1   o r d e r   b y   R e g i o n D e s c r i p   " , t r u e )  
  
 N u m F o r m s   =   G e t V a r ( " d b _ N u m R e c o r d s " )  
  
 L i s t   =   " "  
 f o r   i   =   1   t o   N u m F o r m s  
         D e v i c e N a m e   =   G e t N d x V a r ( " d b _ R e g i o n D e s c r i p " , i )  
           L i s t   =   L i s t   +   C S t r ( D e v i c e N a m e )   +   " \ n "  
 n e x t  
  
 c a l l   S e t V a r ( " R e g i o n L i s t " , L i s t )  
     mE#џўџ	O n   U p d a t e џўџџMU I S t a t e   =   G e t V a r ( " U I S t a t e " )  
  
 I f   U I S t a t e   =   " C o n f i r m T a g C r e a t e "   T h e n  
          
         . C r e a t e T a g M s g 1   =   " T h e   I n s t a n c e   "   &   G e t V a r ( " T a g I D " )   &   "   d o e s   n o t   e x i s t .   "  
         . C r e a t e T a g M s g 2   =   " C l i c k   [ C r e a t e   N e w   I n s t a n c e ]   t o   c r e a t e   a   n e w   i n s t a n c e   i n   P o w e r D B . "  
         . C r e a t e T a g M s g 3   =   " C l i c k   [ B a c k ]   t o   m o d i f y   t h e   I n s t a n c e   a n d   t r y   a g a i n . "  
          
  
 E n d   I f  
      mE#џўџO n   P a g e   S e t   F o c u s џўџ     mE#џўџO n   F o r m   E x i t џўџ     mE#џўџO n   T i m e r џўџ 	    mE#џўџG l o b a l   F u n c t i o n s   & &   V a r i a b l e s џўџ     mE#џўџD a t a џўџ     џўџ  џџ  OPdbDrawText   tE#РРР     з        Д             iF#	             џџ   OOptDrawBorderData                            џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџџџ   OPdbFormsToolText76        77    џџ   OPdbFormsTagInfo        џўџ       Д   q  в   lE#     џўџT a g I D      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     }                     iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ            Џ   lE# 
   T a s k : џўџ      ЭЭЭЭ   qE  џўџT a s k :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#џџџ     в   2     б        iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ       б  b    lE# &   C r e a t e   N e w   I n s t a n c e џўџT a g C r e a t e N e w     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџџi f   I s B l a n k S t r i n g ( G e t v a r ( " R e g i o n s " ) )   T h e n  
         m s g b o x ( " P l e a s e   s e l e c t   R e g i o n   i n   t h e   d r o p d o w n " )  
 E l s e  
  
 R e g i o n s   =   G e t v a r ( " R e g i o n s " )  
  
 S q l   =   " S E L E C T   T o p   1   R e g i o n G U I D   F r o m   P d b R e g i o n s   w h e r e   R e g i o n D e s c r i p   =   ' "   &   R e g i o n s   &   " ' "  
 c a l l   A p p l i c a t i o n . Q u e r y D a t a b a s e ( F o r m . F o r m I d ,   S q l ,   0 )  
          
          
 c a l l   S e t V a r ( " R e g i o n G U I D L i s t " , G e t v a r ( " d b _ R e g i o n G U I D " ) )  
  
 c a l l   S e t V a r ( " S i t e I D " ,   " " )  
 c a l l   S e t S c r e e n ( " L o o k u p S i t e " )  
  
  
 E n d   I f     mE#џўџO n   E n t e r џўџ      mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџC r e a t e   N e w   I n s t a n c e     9у=                                     ЭV#              џўџA r i a l џџџ                tE#РРР     X     О     О           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     О       Љ  lE#     џўџC r e a t e T a g M s g 3     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     Ы  #   F   7   F           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ        џџџџџџџџџџџџ76        77            џўџ     F   7     Z   lE# $   I N S T A N C E   N O T   F O U N D џўџC o n f i r m T a g C r e a t e H e a d i n g      ЭЭЭЭ   qE  џўџI N S T A N C E   N O T   F O U N D     ЋЊ*>                                      ЭV#             џўџA r i a l џџџ                tE#РРР     }        Д             iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ       Д     в   lE#    I n s t a n c e : џўџ      ЭЭЭЭ   qE  џўџ	I n s t a n c e :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     з                     iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ          q  Џ   lE#     џўџT i c k e t      ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#РРР     X     О   m  О           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     О   m      lE#     џўџC r e a t e T a g M s g 2     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ                tE#џџџ     s   2   џ   б  џ       iF#	  АA                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     џ   б  r    lE#    B a c k џўџT a g C r e a t e B a c k     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџc a l l   G o B a c k ( )     mE#џўџO n   E n t e r џўџ      mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџB a c k     9у=                                     ЭV#              џўџA r i a l џџџ                tE#РРР     }        з             iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ       з     ѕ   lE#    R e p o r t   T y p e : џўџ     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџR e p o r t   T y p e :     9у=                                      ЭV#              џўџA r i a l џџџ                tE#РРР     X     О   O  О           iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ     О   O    m  lE#     џўџC r e a t e T a g M s g 1     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ     9у=                                       ЭV#              џўџA r i a l џџџ             џџ  OptDrawDropDowntE#џџџ     з        з             iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџџџ   OPdbFormsToolDropDownz76
                       џўџ       з   q  ѕ   lE#     џўџ
R e p o r t T y p e      mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ     9у=                                        ЭV#              џўџA r i a l џџџ                       ЭЭЭЭџўџ1 0 7 7 0   B A T T E R Y  
 2 0 1 0 0   S I T E S C A N  
 8 0 2 0 0   A P M  
 8 0 1 0 0   A P S  
 8 0 3 0 0   N X   2 2 5 - 6 0 0  
 1 0 7 7 1   B A T T E R Y  
 T H E R M A L   P M V  
 8 0 8 0 0   U P S  
 E H S   S i t e   S u r v e y  
  
     џўџ    ttE#џџџ     з        њ             iF#	                                         џџџџ            џџџџ                џўџ џџџџџџџџ  џ џџџџ    џџџџџџџџџџџџzz76
                       џўџ       њ   q    lE#     џўџR e g i o n s      mE#џўџO n   I n i t i a l   U p d a t e џўџ"C a l l   S e t v a r ( " R e g i o n s " ,   . P d b R e g i o n )     mE#џўџ	O n   U p d a t e џўџ=. t h i s ! l i s t   =   G e t V a r ( " R e g i o n L i s t " )  
         . t h i s ! v i s i b l e   =   t r u e  
      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџ     9у=                                        ЭV#              џўџA r i a l џџџ                       ЭЭЭЭџўџ 
  
     џўџ        tE#РРР     }        њ             iF#	                                             џџџџ            џџџџ                џўџ џџџџџџџџ  џ         џџџџџџџџџџџџ76        77            џўџ       њ       lE#    R e g i o n : џўџ     mE#џўџO n   I n i t i a l   U p d a t e џўџ     mE#џўџ	O n   U p d a t e џўџ      mE#џўџ	O n   S e l e c t џўџ     mE#џўџO n   E n t e r џўџ     mE#џўџO n   R i g h t   C l i c k џўџ 
    ЭЭЭЭ   qE  џўџR e g i o n :     9у=                                      ЭV#              џўџA r i a l џџџ             џўџD a t e M o d i f i e d        q=
Мф@џўџD a t e C r e a t e d        jэZ>зу@џўџ	D a t e S y n c h        jэZ>зу@џўџb I s D e l         џўџ
D e l e t e I n f o    џўџ џўџ
b I s R e t i r e d        џўџb O p t i m a F o r m        џўџb S u g g e s t U p d a t e        џўџ	L a s t M o d B y    џўџ џўџC r e a t o r G u i d    џўџ џўџT e m p l a t e s U s e d    џўџ8U s e r   D a t a | O p t i m a   D a t a   -   D o   N o t   C h a n g e | C u s t o m   F i e l d   E n t r y џўџE x t e r n a l T e m p l a t e s    џўџ џўџT e m p l a t e F a m i l y    џўџ џўџT e m p l a t e M a n u    џўџ џўџb I s L o c a l F o r m         џўџS y n c h W r i t e D a t e        Gь;ЖТ+п@џўџS t a t F l g         џўџb L o c k e d        џўџL o c k I n f o    џўџ    џўџ 
 