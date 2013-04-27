use strict;
use warnings;

print <<END;
#include <stdint.h>

/*
  These lookup tables are used to do quick conversion from a packed 12-bit RGB
  pixel in the bitmap to the 36 bits (3*12) of shift-out data to be sent to the
  TLCs.

  We use two tables - one for even indexes (starting at a byte boundary) and
  one for odd indexes (starting at a four-bit boundary within the byte).

  The input to the lookup-table is a 12-bit packed RGB pixel. Red is in bits
  0-3, green in bits 4-7, and blue in bits 8-11.

  The output is the 36 bits of data that needs to be shifted out to the TLC5940
  LED drivers. There are two 32-bit words for each entry. The bytes within each
  32-bit word are stored little-endian, so that simple unaligned memory writes
  on a little-end architecture will work to write them to the memory buffer.

  The _odd variant of the lookup table has data shifted 4 bits so that it can
  be joined with the _even variant to fill the output buffer without need to
  shift data around.

  The data shifted out to the TLC5940 ICs is a sequence of 3 12-bit words, each
  12-bit word stored in big-endian fashion. First blue, then red, then green.

  As a result, this is the format of an "even" lookup (in |A B|, A are the high
  4 bits):

    | blue[11-4] | blue[3-0] red[11-8] | red[7-0] | green[11-4] |
    | green[3-0] 0 | 0 | 0 | 0 |

  And here an "odd" lookup:

    | 0 blue[11-8] | blue [7-0] | red[11-4] | red[3-0] green[11-8] |
    | green[7-0] | 0 | 0 | 0 |

  Due to little-endian byte order, | A | B | C | D | becomes as an uint32_t:
  A | B<<8 | C<<16 | D<< 24.

  The mapping from 4-bit colour in the bitmap to 12-bit PWM value in the
  TLC5940 is non-linear to get better dynamic range.
*/

static const uint32_t tlc_lookup_even[0x1000][2] = {
END

my @col_4to12 = (0, 23, 89, 192, 332, 508, 719, 964,
                 1242, 1554, 1898, 2275, 2684, 3125, 3597, 4095);
my @even;
my @odd;
for (my $i = 0; $i < 0x1000; ++$i) {
  my $r = $col_4to12[$i & 0xf];
  my $g = $col_4to12[($i >> 4) & 0xf];
  my $b = $col_4to12[$i >> 8];
  push @even,
    [($b>>4) | (($b & 0xf)<<4 | ($r>>8)) << 8 | ($r&0xff) << 16 | ($g>>4) << 24,
     ($g&0xf)<<4];
  push @odd,
    [($b>>8) | ($b&0xff) << 8 | ($r>>4) << 16 | (($r&0xf)<<4 | $g>>8) << 24,
     $g & 0xff];
}
output_array(\@even);
print <<END;
};

static const uint32_t tlc_lookup_odd[0x1000][2] = {
END
output_array(\@odd);
print <<END;
};
END

sub output_array {
  my ($a) = @_;
  for (my $i = 0; $i < @$a; ++$i) {
    print "  " if (0 == ($i % 4));
    printf "{0x%x,0x%x}", $a->[$i][0], $a->[$i][1];
    print "," unless $i == scalar(@$a);
    print (($i%4) == 3 ? "\n" : " ");
  }
};
