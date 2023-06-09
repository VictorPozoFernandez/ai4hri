print('''------------------------------------------
 \033[94m- hi, in which colors is available the canon model? (Customer) \033[0m

------------------------------------------
 \033[94m- this only comes in black. which type of camera are you looking for? (Shopkeeper) \033[0m

Detected topics: ['Color']
Detected model: ['3', 'Canon EOS 5D Mark III']

\033[92mSHOPKEEPER IS RIGHT\033[0m
Product: Canon EOS 5D Mark III
Feature: Color
Reason: The Canon EOS 5D Mark III is indeed available in black''')
      
input()

print('''------------------------------------------
 \033[94m- something that's light and attractive.  (Customer) \033[0m

------------------------------------------
 \033[94m- we have a very lightweight camera that has a lot of presets, please come and have a look at this nikon here (Shopkeeper) \033[0m

Detected topics: ['Weight', 'Preset_modes', 'Type_of_camera', 'Model']
Detected model: ['1', 'Nikon Coolpix S2800']

\033[92mSHOPKEEPER IS RIGHT\033[0m
Product: Nikon Coolpix S2800
Feature: Weight
Reason: The Nikon Coolpix S2800 is indeed lightweight, weighing 120 grams

\033[92mSHOPKEEPER IS RIGHT\033[0m
Product: Nikon Coolpix S2800
Feature: Preset_modes
Reason: The Nikon Coolpix S2800 has 18 preset modes''')

input()

print('''------------------------------------------
 \033[94m- I want good lighting and loss of motion I guess (Customer) \033[0m

------------------------------------------
 \033[94m- It is very lightweight, 120 grams, and it has 18 preset modes (Shopkeeper)\033[0m 


Detected model: ['1', 'Nikon Coolpix S2800'] (They keep talking about the same camera)
Detected topics: ['Weight', 'Preset_modes']

\033[92mSHOPKEEPER IS RIGHT\033[0m
Product: Nikon Coolpix S2800
Feature: Weight
Reason: The Nikon Coolpix S2800 does weigh 120 grams

\033[92mSHOPKEEPER IS RIGHT\033[0m
Product: Nikon Coolpix S2800
Feature: Preset_modes
Reason: The Nikon Coolpix S2800 does have 18 preset modes''')

input()

print('''------------------------------------------
 \033[94m- that sounds great how much does this cost? (Customer) \033[0m

------------------------------------------
 \033[94m- $68 and it gets very high-resolution photographs too, it's right value for the price (Shopkeeper) \033[0m

Detected model: ['1', 'Nikon Coolpix S2800'] (They keep talking about the same camera)
Detected topics: ['Price', 'Resolution']

\033[92mSHOPKEEPER IS RIGHT\033[0m
Product: Nikon Coolpix S2800
Feature: Price
Reason: The Nikon Coolpix S2800 is indeed priced at $68

------------------------------------------
 \033[94m- how many megapixels (Customer)\033[0m 

------------------------------------------
 \033[94m- 20.1 megapixels (Shopkeeper) \033[0m

Detected model: ['1', 'Nikon Coolpix S2800'] (They keep talking about the same camera)
Detected topics: ['Resolution']

\033[92mSHOPKEEPER IS RIGHT\033[0m
Product: Nikon Coolpix S2800
Feature: Resolution
Reason: The Nikon Coolpix S2800 indeed has a resolution of 20.1 megapixels''')
      
input()

print('''\033[94m- The Nikon costs 550 dollars and the Sony, 68 (Shopkeeper)\033[0m 

Detected topics: ['Price']
Detected model: ['1', 'Nikon Coolpix S2800', '2', 'Sony Alpha a6000']

\033[91mSHOPKEEPER IS MISTAKEN\033[0m
Product: Nikon Coolpix S2800
Feature: Price
Reason: The Nikon Coolpix S2800 actually costs 68 dollars, not 550

\033[91mSHOPKEEPER IS MISTAKEN\033[0m
Product: Sony Alpha a6000
Feature: Price
Reason: The Sony Alpha a6000 actually costs 550 dollars, not 68




''')






