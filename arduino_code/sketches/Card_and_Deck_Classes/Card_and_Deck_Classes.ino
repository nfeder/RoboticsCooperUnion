//create a card class
class Card {
  //private variables:
  char suit;
  int value;

  //public functions:
  public:
  //create a constructor to set up the variables
  Card(char, int);
  //create getter methods to return the variables' values
  char getSuit(){
    return suit;
  }
  int getValue() {
    return value;
  }
};
//define the constructor
Card::Card(char s, int v) {
  suit = s;
  value = v;
}

//create a deck class
class Deck {
  //private variables:
  Card *cards[52];
  int topCard;
  
  //public functions:
  public:
  //create a constructor to set up each individual Card
  Deck() {
    char s = 'x';
    for (int i = 0; i < 52; i++) {
      for (int j = 0; j < 4; j++) {
        for (int v = 0; v < 13; v++) {
          switch(j) {
            case 0:
             s = 'H';
             break;
           case 1:
             s = 'C';
             break;
           case 2:
             s = 'S';
             break;
           case 3:
             s = 'D';
             break;
          }

         cards[i] = new Card(s, v);
       }
      }
    }
    topCard = 0;
  }
  
  //create a method to return the top card, then 'flip' to the next card
  Card getTopCard() {
    Card tc = *cards[topCard];
    if (topCard < 51) {
     topCard++;
    }
    else {
      topCard = 0;
    }
    return tc;
  }

  //reset the deck by re-calling the constructor
  void reset() {
    Deck();
  }

  //shuffle the deck
  void shuffle() {
    for (int i = 0; i < 52; i++) {
      int otherCard = random(52);
      Card temp = *cards[otherCard];
      cards[otherCard] = cards[i];
      *cards[i] = temp;
    }
  }
};

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:

}
