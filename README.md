# Riešitelia
1. meno1
2. meno2
3. meno3

# Bonusová úloha VRS (4b za 30min)
S použitím GitHub nástrojov vytvoríte rovnaký program ako v zadaní 1. Zadanie pozostáva zo 4 častí, za každú môžete dostať 1 bod pre tím, ktoré si môžete ľubovoľne prerozdeliť vrámci tímu.

1. Vytvoríte si vlastný hlavný repozitár (nie Fork) z `https://github.com/stecf/bonus_13`
2. Do `README.md` na začiatok dopíšete mená riešiteľov
3. Pre každý zo štyroch problémov vytvoríte:
	1. Issue, kde bude názov problému, napr. `Missing macros to the MCU's registers` a krátky popis problémun, napr. `Need macros to the MCU's registers so they can be used in main.c and make LED blink application code readable and great again!`
	2. Feature branch napr. `adding_register_macros`, kde zapracujete zmeny kódu a nahráte túto vetvu na vzdialený repozitár (nezabudnite dať deskriptívny popis commit-u)
	3. Pull Request na pridanie zmien z Feature branch do `main` branch vo vašom Fork s krátkym popisom zmien, napr. `Added macros to the MCU's registers.`
	4. K Pull Request priradíte Reviewera a riešenú Issue (napravo Reviewer a Development)
	5. Reviewer skontroluje kód a ak je s ním spokojný, tak ho schváli (Approve)
	6. Contributor po schválení PR dokončí merge do `main` vetvy

# Riešené problémy
1. Chýbajú makrá registrov MCU pre prístup ku GPIOA-3 (tlačidlo) a GPIOA-4 (LED)
2. Chýba konfigurácia GPIOA-3 (tlačidlo) a GPIOA-4 (LED)
3. LED má blikať rýchlo pri stlačenom tlačidle, inak pomaly
4. Chýba [licenčný súbor BSD-3](https://opensource.org/license/bsd-3-clause/) a nastavenie .gitignore na ignorovanie `build` a `.settings` priečinkov.

# Vytvorenie vlastného repozitára z iného
Vytvorte si najprv vlastný repozitár na GitHube, do ktorého budete chciet spraviť kópiu tohto repozitára.

```bash
# Naklonovanie zdrojoveho repozitára
git clone https://github.com/stecf/bonus_13

# Vymazanie predchádzajúceho git a vytvorenie nového
cd bonus_13
rm -rf .git
git init

# Vytvorenie prvého commit-u
git add .
git commit -m "Initial commit"

# Premenovanie hlavnej vetvy a nahratie na remote repository
git branch -M main
git remote add origin <path to remote repository>
git push -u origin main
```
