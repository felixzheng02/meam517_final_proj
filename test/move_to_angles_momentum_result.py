from utils import plot
import numpy as np
import matplotlib.pyplot as plt

theta_no_momentum = [0.0, 0.0, 8.081784143338886e-05, 0.0005444637819082436, 0.001489124654509174, 0.003073929423759198, 0.005456723424098649, 0.00879262955815215, 0.013233640785067636, 0.01892730013538224, 0.02601514131715361, 0.03463088356275592, 0.044898371356777836, 0.056929096689694134, 0.0708687536889521, 0.08679060181926836, 0.10474053649547553, 0.12474799791735225, 0.1468343534092237, 0.17064845548165936, 0.19540831350119486, 0.2212867799712534, 0.2477479619517276, 0.2743124002284954, 0.3005162967692185, 0.3263204837500646, 0.3516943483499155, 0.37404048168092063, 0.3950635145297475, 0.41471738772483274, 0.433481791132697, 0.4519873127067479, 0.4685127063316999, 0.48450989348132745, 0.4999477122116028, 0.5148474134932797, 0.5292497108743127, 0.5432026140009425, 0.5567563981067835, 0.5699615556208041, 0.582868034103894, 0.5955250013792578, 0.6079808443620512, 0.6202832648779929, 0.6324794182702073, 0.6446160689920608, 0.6567397537335223, 0.6688969477233271, 0.6811342324663773]
theta_tar_no_momentum = [0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977]
theta_momentum = [0.0, 0.0, 7.083647039664176e-05, 0.0007781808513606073, 0.0017186303296257283, 0.0028127474484431515, 0.003985782442676946, 0.005165336808159743, 0.006343959040527009, 0.007516687052574052, 0.008224847416876117, 0.008641028223341597, 0.008693030238068293, 0.008308715538536477, 0.0074146724638436655, 0.006738243939307085, 0.006646175197825416, 0.007506932693349521, 0.009690968594122513, 0.013125965919466714, 0.017742982780572583, 0.023476614244700603, 0.03026578591933543, 0.03805449902081787, 0.04679254002607562, 0.05643615831201766, 0.06694871657500207, 0.07830131994512088, 0.09047340935015191, 0.10345334618446782, 0.11723897843915038, 0.1318322314388122, 0.14557338663626285, 0.15759011596891923, 0.16754706261928404, 0.1754854221099298, 0.18142442787793214, 0.18537852586155795, 0.18735771547754995, 0.18736754580491627, 0.18540909027933206, 0.18147903454662, 0.175788247725881, 0.16854200640640998, 0.1599460007631366, 0.15018685806087384, 0.1394501547133488, 0.12791424171462284, 0.11574989443048159, 0.10311976902259734, 0.09017804110435772, 0.0770700696150067, 0.06393211629361639, 0.05088907397675686, 0.03805791999968351, 0.025546918745722576, 0.013452827113117332, 0.00186297727207777, -0.009144314898439277, -0.019499304094100906, -0.029140286174408246, -0.03801697112217821, -0.046075196569469554, -0.05327194718644257, -0.05957400291158414, -0.0649502672516836, -0.06935990684985499, -0.07279335514206588, -0.07522882399706632, -0.07664532765227713, -0.07702277178813147, -0.07634210598096956, -0.0745969160653553, -0.07196998300986532, -0.06869174941267216, -0.0650107523399962, -0.06119466932513354, -0.057540502311063745, -0.05420922591937827, -0.051375810713653255, -0.049228077513555346, -0.04796505186250763, -0.047794561856613745, -0.048930112166382676, -0.05158669539907017, -0.055724035779156855, -0.06130496186326979, -0.06829644609666963, -0.0766675386836166, -0.08638959384147428, -0.0974477107258937, -0.10983481637267786, -0.1236949786037433, -0.1372128675298093, -0.14966774078001813, -0.16003711080632185, -0.16837443500384366, -0.17476244822322387, -0.179126036552834, -0.18152346597716512, -0.18195034128448292, -0.18042942454733554, -0.17694772763315875, -0.17171337998941752, -0.16492862482591922, -0.15678831427099135, -0.14748758970206546, -0.13719156003677424, -0.12608225478085364, -0.11433050091108815, -0.10208114966449416, -0.08950035817494748, -0.07673382102026453, -0.06393401369771404, -0.05121646147020513, -0.03869075451628975, -0.02646368101660566, -0.014632994807661677, -0.0032872822382131344, 0.007491717764708999, 0.017635189232482747, 0.02707452611987918, 0.03575315874619959, 0.04362160494079409, 0.05063620520763745, 0.05675961565078516, 0.0619591938160872, 0.06620551906730725, 0.06947197971039774, 0.07173399505390426, 0.0729682302130816, 0.07315179321795505, 0.07226334871875119, 0.07027835037544006, 0.06743791478790398, 0.06399719681645409, 0.06023299884847213, 0.05644871826162885, 0.052979442462784895, 0.05000674779132899, 0.047721146476026134, 0.04632586252702066, 0.04603407866967849, 0.04706550463550583, 0.04964215277590432, 0.053722572194011435, 0.05926897783459287, 0.06624706156807877, 0.07462759031064864, 0.08438738658453658, 0.09551033219172544, 0.10798835392557742, 0.1218224338100937, 0.1356783457200279, 0.14812018552391246, 0.15857984695454216, 0.16703491499723472, 0.17354395284894944, 0.17804339689830137, 0.18056889412035482, 0.18111960394339122, 0.17969403562189054, 0.17629019419085956, 0.17111678613754616, 0.16437201448175737, 0.15625129152444864, 0.14694226490039874, 0.13660651914220462, 0.12543251102585354, 0.11360824386285111, 0.10130318618963703, 0.08866680117364052, 0.07582579952215553, 0.06292030848234989, 0.050076126059124856, 0.03740918010093597, 0.02502531968203196, 0.013020973087593242, 0.0014804900699774207, -0.009515975820727909, -0.019898849382888287, -0.029601486125891117, -0.038573031538090996, -0.046753043728568255, -0.05411263921889536, -0.06061721685359834, -0.06622864179575816, -0.0709272219163438, -0.07469670160108949, -0.07750550394285574, -0.07933997322114619, -0.08018455133707215, -0.08001855732305846, -0.07882143973589903, -0.07677846028363713, -0.0740849257328451, -0.0709498765753393, -0.06760006305815276, -0.06428454129486497, -0.06128108816682982, -0.0587403375533194, -0.056823965212010365, -0.05570355097990113, -0.055558349359688765, -0.05657246072851466, -0.05893105614564722, -0.0625986600620855, -0.06754366648147545, -0.07373725645293032, -0.08115516466405251, -0.08977855518066097, -0.09959487740939464, -0.11059870352431958, -0.12279254939857658, -0.13456228013559232, -0.14506811875538864, -0.15349176075195892, -0.1598901599413445, -0.1642858282413365, -0.16669764051085603, -0.16711893111812978, -0.16554850281354067, -0.16198209592924592, -0.1566150916754227, -0.14963956487904412, -0.14123793898324824, -0.1315857623288258, -0.12085839782817867, -0.10921718965346186, -0.09681642863975315, -0.08380473629827392, -0.07031985778180165, -0.05649094524083522, -0.04243771124844976, -0.028171865556446428, -0.01399574025445969, 0.00010088860147331266, 0.014046767862416619, 0.02777586167941821, 0.041252273344742824, 0.05434645568117199, 0.06706259297106486, 0.07942572034897061, 0.0914156557990183, 0.10299519468663305]
theta_tar_momentum = [0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, -0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977, 0.10471975511965977]
time_steps_momentum = range(len(theta_momentum))
time_steps_no_momentum = range(len(theta_no_momentum))

plt.figure(figsize=(12, 8))

plt.plot(time_steps_momentum, theta_momentum, label='theta with momentum', color='royalblue', linestyle='-')
plt.plot(time_steps_no_momentum, theta_no_momentum, label='theta without momentum', color='grey', linestyle='-')
plt.plot(time_steps_momentum, theta_tar_momentum, label='theta target with momentum', color='royalblue', linestyle='--')
plt.plot(time_steps_no_momentum, theta_tar_no_momentum, label='theta target without momentum', color='grey', linestyle='--')
# Add labels, legend, and show
plt.xlabel('Time Steps', fontsize=18)
plt.ylabel('Angle (rad)', fontsize=18)
plt.legend(fontsize=18)
plt.grid(True)
plt.tight_layout()
plt.show()