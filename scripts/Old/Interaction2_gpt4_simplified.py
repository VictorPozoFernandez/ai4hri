print('''------------------------------------------
\033[94m - good afternoon how can I help (Shopkeeper)\033[0m

Detected topics: ['NULL']
Detected model: ['NULL']

------------------------------------------
\033[94m - how does this canon camera perform on low lighting? (Customer) \033[0m

------------------------------------------
\033[94m - this one is the best camera for low light. It has 6400 ISO, and it has silent shooting. It has little noise. (Shopkeeper) \033[0m

Detected topics: ['ISO', 'Silent_Shooting']
Detected model: ['3', 'Canon EOS 5D Mark III']

\033[92mSHOPKEEPER IS RIGHT\033[0m
Product: Canon EOS 5D Mark III
Feature: ISO
Reason: The Canon EOS 5D Mark III does have an ISO of up to 6400

\033[92mSHOPKEEPER IS RIGHT\033[0m
Product: Canon EOS 5D Mark III
Feature: Silent_Shooting
Reason: The Canon EOS 5D Mark III does have silent shooting

------------------------------------------
\033[94m - that sounds great what about the price? (Customer) \033[0m

------------------------------------------
\033[94m - it is a high end professional camera, the body itself is $2000 (Shopkeeper) \033[0m

Detected topics: ['Price']
Detected model: ['3', 'Canon EOS 5D Mark III'] (They keep talking about the same camera)

\033[92mSHOPKEEPER IS RIGHT\033[0m
Product: Canon EOS 5D Mark III
Feature: Price
Reason: The Canon EOS 5D Mark III body is indeed priced at $2000

------------------------------------------
\033[94m - I see, do you have anything in this type that's more reasonably priced? (Customer) \033[0m

------------------------------------------
\033[94m - yes of course, come and have a look at the Sony it cost $550 (Shopkeeper) \033[0m

Detected topics: ['Price']
Detected model: ['2', 'Sony Alpha a6000']

\033[92mSHOPKEEPER IS RIGHT\033[0m
Product: Sony Alpha a6000
Feature: Price
Reason: The shopkeeper correctly states the price of the Sony Alpha a6000''')

