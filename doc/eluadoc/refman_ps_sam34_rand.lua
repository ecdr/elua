-- eLua reference manual - platform data

data_en = 
{

  -- Title
  title = "eLua reference manual - SAM34 Random Sequence Generator",

  -- Menu name
  menu_name = "rand",

  -- Overview
  overview = [[This module contains functions to use the random sequence generator.</p>
<p>Returns numbers in a random sequence, uniformly distributed over the given range.
(Often called a "random number generator," although that is ambiguous, 
it is the sequence of numbers that is random or unpredictable.)]],

   -- Data structures, constants and types
  structures = 
  {
    { text = [[MAX]],
      name = "rand constants",
      desc = [[Largest possible integer returned by rand.next.]]       
    }
  },

  -- Functions
  funcs = 
  {
    { sig = "#sam34.rand.next#()",
      desc = "Get a new integer at random.",
      ret = "$num$ - An integer chosen at random in the range 0 to $sam34.rand.MAX$.",
    },

    { sig = "#sam34.rand.fnext#()",
      desc = "Get a new floating point number at random.",
      ret = "$num$ - A floating point number chosen at random in the range (0.0 to 1.0).",
    },
  },
}

data_pt = data_en
