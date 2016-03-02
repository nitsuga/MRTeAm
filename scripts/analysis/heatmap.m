%--
% heatmap.m
% input data file in the following format:
% columns:
%  1 = RR
%  2 = OSI
%  3 = SSI
%  4 = PSI
%  5 = condition
% rows:
%  1 -4: MR-CT-DA
%  5 -8: MR-IT-DA
%  9-12: SR-CT-DA
% 13-16: SR-IT-DA
%--

function heatmap( prefix )
%prefix = 'NAP_TIME'

%- open input file
infilename = sprintf( '%s-rank.dat', prefix );
d = load( infilename );

%-define labels
mylabels = [ 'MR-CT-DA'; 'MR-IT-DA'; 'SR-CT-DA'; 'SR-IT-DA' ];
mystart  = [ 1, 5, 9, 13 ];

%-define colors
mycolors = [ 0.1, 0.1, 0.1; 0.3, 0.3, 0.3; 0.6, 0.6, 0.6; 0.9, 0.9, 0.9 ];

%-loop through four conditions at a time
for s=1:4
  figure
%-each column represents one mechanism (RR, OSI, SSI, PSI)
  set( gca,'xlim',[0 4] );
%-each row represents one condition
  set( gca,'ylim',[0 4] );
  set( gca,'dataaspectratio',[1 1 1] )
  hold on;
  %-loop through 4 conditions (in rows)
  c=1;
  for p=mystart(s):mystart(s)+3
    %-loop through 4 mechanisms (in columns)
    for i=1:4
      fprintf( 'p=%d i=%d rank=%d\n', p, i, d(p,i) );
      x = [i-1 i   i     i-1   i-1];
      y = [4-c 4-c 4-c+1 4-c+1 4-c];
      clr = mycolors( d(p,i),: );
      patch( x, y, clr );
    end
    c=c+1;
  end
  set( gca, 'xtick', [0.5 1.5 2.5 3.5] )
  set( gca, 'xticklabel', ['RR ';'OSI';'SSI';'PSI'] )
  set( gca, 'ytick', [0.5 1.5 2.5 3.5] )
  set( gca, 'yticklabel', ['di-s2';'cl-s2';'di-s1';'cl-s1'] )

  %set( gca, 'FontSize', 28 )

  xticklabel = get(gca,'XTickLabel');
  set(gca, 'XTickLabel', xticklabel, 'fontsize', 30)

  yticklabel = get(gca,'YTickLabel');
  set(gca, 'YTickLabel', yticklabel, 'fontsize', 30)
  
  outfilename = sprintf( '%s-%s-heatmap.png', prefix, mylabels(s,:) );
  print( '-dpng', outfilename );
  hold off;
end
