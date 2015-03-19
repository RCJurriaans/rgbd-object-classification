dictionary
  * Per plaatje, maak histogram van alle descriptors, met op de x-as de dictionary elements. normalize dit.
  * Voor elke class, train een supprt vector machine met de histogrammen van die class als positieve punten, en de histogrammen van de andere classes als negatieve data punten.
  * Voor nieuwe plaatjes, gebruik dezelfde procedure om plaatjes te classifyen met de SVM als behorend tot die class of niet.