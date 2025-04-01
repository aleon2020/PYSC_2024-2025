(define (problem robot_problem)
(:domain robot_library)
(:objects 
  peepo - robot
  entrance origin - hall
  large_table1 large_table2 rubik_table big_table1 big_table2 - table
  bookshelf1 bookshelf2 bookshelf3 bookshelf4 bookshelf5 bookshelf6 bookshelf7 bookshelf8 - bookshelf
  book1 book2 book3 book4 - book
  rubik_cube trash - miscellaneus
  visitor1 - visitor
)
(:init 
  (robot_at peepo origin)
  (robot_not_busy peepo)

  (gripper_free peepo)

  (object_at book1 bookshelf1)
  (object_at book2 bookshelf5)
  (object_at book3 large_table1)
  (object_at book4 large_table2)
  (object_at rubik_cube rubik_table)
  (object_at trash big_table1)
  
  (visitor_at visitor1 entrance)

  (noise_at large_table1)
  (noise_at large_table2)

  (connected origin entrance)
  (connected entrance large_table2)
  (connected large_table2 entrance)
  (connected entrance rubik_table)
  (connected rubik_table entrance)
  (connected entrance bookshelf1)
  (connected bookshelf1 entrance)
  (connected entrance big_table1)
  (connected big_table1 entrance)
  (connected large_table1 bookshelf3)
  (connected bookshelf3 large_table1)
  (connected large_table1 bookshelf4)
  (connected bookshelf4 large_table1)
  (connected large_table1 big_table1)
  (connected big_table1 large_table1)
  (connected large_table2 rubik_table)
  (connected rubik_table large_table2)
  (connected large_table2 big_table1)
  (connected big_table1 large_table2)
  (connected bookshelf5 bookshelf6)
  (connected bookshelf6 bookshelf5)
  (connected bookshelf6 bookshelf7)
  (connected bookshelf7 bookshelf6)
  (connected bookshelf7 big_table1)
  (connected big_table1 bookshelf7)
  (connected big_table1 big_table2)
  (connected big_table2 big_table1)
)
(:goal 
  (and
    (book_found book1 bookshelf1)
    (book_found book2 bookshelf6)
    (object_at book3 bookshelf1)
    (object_at book4 bookshelf6)
    (visitor_at visitor1 large_table1)
    (solved rubik_cube)
    (object_at trash entrance)
    (quiet large_table2)
    (quiet large_table1)
  )
)
)